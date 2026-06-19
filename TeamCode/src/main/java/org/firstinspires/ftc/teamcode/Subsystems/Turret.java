package org.firstinspires.ftc.teamcode.Subsystems;

import static com.pedropathing.math.MathFunctions.clamp;
import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.Calculos.encoderTicksToAngle;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.AutoConstants;
import org.firstinspires.ftc.teamcode.Constants.AutoPoses;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class Turret implements Subsystem {

    public static final Turret INSTANCE = new Turret();

    public static ControlSystem controllerauto;

    public static double Tkp  = 0.008;
    public static double Tki  = 0.000000000003;
    public static double Tkd  = 0.0002;
    public static double offsetx  = 3;
    public static double shootOffset = 0.0;
    public static boolean shootBrakeEnabled = false;
    private static double shootBrakeAngle = 0.0;
    private double headingRad = 0.0;
    private double compensation = 0.0;
    private double robotX     = 250;
    private double robotY     = 210;
    public static double angleLL                    = 0.0;
    public static double turretAngle                = 0.0;
    public static double destinationAngle           = 0.0;
    public static double calculatedDestinationAngle = 0.0;
    public static double toTurn                     = 0.0;
    public static double goalx = 144;
    public static double goaly = 0;
    public static double offset         = 0;
    public static double RecoveryOffset = 0.0;

    public static double  manualPower     = 0.6;
    public static boolean manualMode      = false;
    public static double  manualDirection = 0.0;
    public static double headingOffset = 0;
    public static boolean followEnabled    = true;
    public static boolean headingEnabled   = true;
    public static boolean lockAngleEnabled = false;
    public static double  lockedAngle      = 0.0;
    private static boolean shootLockEnabled = false;
    public static boolean enabled;
    private static double shootLockAngle = 0.0;
    private double lockedHeadingRad = 0.0;
    private double resetTick = 0.0;
    private static final MotorEx motor = new MotorEx("turret").brakeMode();

    private static final double max =  90;
    private static final double min = -270;
    public enum SnapPosition { SHOT1, SHOT2 }
    public static double snap1Angle       = 0.0;
    public static double snap2Angle       = 45.0;
    public static double snapTolerance    = 3.0;
    private static boolean      snapModeEnabled  = false;
    private static double       snapTargetAngle  = 0.0;
    private static boolean      snapBrakeActive  = false;
    private static double       snapBrakeAngle   = 0.0;

    private Turret() {}
    public static void enableSnapMode(SnapPosition position) {
        snapTargetAngle = (position == SnapPosition.SHOT1) ? snap1Angle : snap2Angle;
        snapModeEnabled  = true;
        snapBrakeActive  = false;
        followEnabled    = false;
        manualMode       = false;
    }

    public static void disableSnapMode() {
        snapModeEnabled = false;
        snapBrakeActive = false;
        followEnabled   = true;
    }

    public static boolean isSnapBrakeActive() {
        return snapModeEnabled && snapBrakeActive;
    }

    public static boolean isSnapReady() {
        if (!snapModeEnabled) return false;
        double error = Math.abs(Math.IEEEremainder(snapTargetAngle - turretAngle, 360.0));
        return error < snapTolerance;
    }

    public void setPoseTracker(
            double robotX,
            double robotY,
            double headingRadians,
            double angleLL,
            boolean azul,
            Telemetry telemetry,
            double compensation
    ) {
        if (shootLockEnabled) return;

        this.robotX       = robotX;
        this.robotY       = robotY;
        this.headingRad   = headingRadians;
        Turret.angleLL    = angleLL;
        this.compensation = compensation;

        if (azul) {
            goalx = AutoPoses.goalPoseazul.getX();
            goaly = AutoPoses.goalPoseazul.getY();
        } else {
            goalx = AutoPoses.goalPoseazul.mirror().getX();
            goaly = AutoPoses.goalPoseazul.mirror().getY();
        }
    }
    public static void addOffset(double angle) { offset += angle; }
    public static void addRecOffset()           { RecoveryOffset += 15; }
    public static void lessRecOffset()          { RecoveryOffset -= 15; }
    public static void stopRecOffset()          { RecoveryOffset = 0; }
    public static void enableShootBrake() {
        shootBrakeEnabled = true;

        turretAngle = encoderTicksToAngle(
                motor.getCurrentPosition() - INSTANCE.resetTick
        );

        shootBrakeAngle = turretAngle;
        motor.brakeMode();
    }

    public static void disableShootBrake() {
        shootBrakeEnabled = false;
    }
    public static void setManualMode(boolean enabled, double direction) {
        manualMode      = enabled;
        manualDirection = direction;
        if (enabled) followEnabled = false;
    }
    public static void enableShootLock(double angle) {
        shootLockEnabled = true;
        shootLockAngle = angle;
    }

    public static void disableShootLock() {
        shootLockEnabled = false;
    }
    public static void lockCurrentAngle() {

        double currentHeadingRad = INSTANCE.headingRad;
        turretAngle  = encoderTicksToAngle(motor.getCurrentPosition() - INSTANCE.resetTick);
        lockedAngle  = turretAngle + Math.toDegrees(currentHeadingRad);
        INSTANCE.lockedHeadingRad = currentHeadingRad;
        lockAngleEnabled = true;
        followEnabled    = true;
    }

    public static void unlockAngle()    { lockAngleEnabled = false; }
    public static void disableHeading() { headingEnabled = false; }
    public static void enableHeading()  { headingEnabled = true; }

    public static boolean isReadyToShoot(double angleTolerance) {
        double angleError = Math.abs(Math.IEEEremainder(destinationAngle - turretAngle, 360.0));
        return angleError < angleTolerance;
    }

    @Override
    public void initialize() {
        controllerauto = ControlSystem.builder()
                .posPid(Tkp, Tki, Tkd)
                .build();

        headingEnabled   = true;
        manualMode       = false;
        lockAngleEnabled = false;
        followEnabled    = true;
        headingOffset    = 0;
        offset           = 0;
        RecoveryOffset   = 0.0;
        shootOffset      = 0.0;
        snapModeEnabled  = false;
        snapBrakeActive  = false;
        motor.setPower(0);
        //resetTurret();
    }

    public void reset() {
        motor.zeroed();
        resetTick        = 0.0;
        headingEnabled   = true;
        manualMode       = false;
        lockAngleEnabled = false;
        followEnabled    = true;
        snapModeEnabled  = false;
        snapBrakeActive  = false;
    }
    public static boolean isAtShootAngle(double tolerance) {
        double error = Math.IEEEremainder(shootLockAngle - turretAngle, 360.0);
        return Math.abs(error) < tolerance;
    }
    public void resetTurret() {
        resetTick = motor.getCurrentPosition();
    }

    public double aimToObject(double currentTurretAngle) {

        if (lockAngleEnabled) {
            destinationAngle = lockedAngle;

            double error = destinationAngle - currentTurretAngle;
            error = Math.IEEEremainder(error, 360.0);

            return error;
        }

        calculatedDestinationAngle = Math.toDegrees(
                Math.atan2(goaly - robotY, goalx - robotX));

        double dx = goalx - robotX;
        double dy = goaly - robotY;
        double dist = Math.sqrt(dx * dx + dy * dy);

        double offsetxturret = offsetx * Math.sin(headingRad);
        double robotAngleDeg = Math.toDegrees(headingRad);

        destinationAngle =
                calculatedDestinationAngle
                        + compensation
                        + RecoveryOffset
                        + offsetxturret;

        double error =
                destinationAngle
                        - (robotAngleDeg + headingOffset)
                        - currentTurretAngle;

        error = Math.IEEEremainder(error, 360.0);

        double projectedAngle = currentTurretAngle + error;
        if (projectedAngle > max) {
            error = max - currentTurretAngle;
        } else if (projectedAngle < min) {
            error = min - currentTurretAngle;
        }

        if (shootLockEnabled) {
            return 0;
        }

        return error;
    }

    @Override
    public void periodic() {

        if (!enabled) {
            motor.setPower(0);
            return;
        }


        if (shootBrakeEnabled) {

            double currentTick = motor.getCurrentPosition() - resetTick;

            turretAngle = encoderTicksToAngle(currentTick);

            double targetAngle = shootBrakeAngle + shootOffset;

            double error = targetAngle - turretAngle;
            error = Math.IEEEremainder(error, 360.0);

            double targetTick = currentTick + angleToEncoderTicks(error);

            controllerauto.setGoal(new KineticState(targetTick));

            double power = controllerauto.calculate(
                    new KineticState(
                            currentTick,
                            motor.getVelocity()
                    )
            );

            motor.setPower(clamp(power, -1, 1));

            return;
        }

        if (shootLockEnabled) {
            double currentTick = motor.getCurrentPosition() - resetTick;

            turretAngle = encoderTicksToAngle(currentTick);

            double error = shootLockAngle - turretAngle;
            error = Math.IEEEremainder(error, 360.0);

            double targetTick = currentTick + angleToEncoderTicks(error);

            double maxTick = angleToEncoderTicks(max);
            double minTick = angleToEncoderTicks(min);

            if (targetTick > maxTick) {
                targetTick -= angleToEncoderTicks(360.0);
            } else if (targetTick < minTick) {
                targetTick += angleToEncoderTicks(360.0);
            }

            targetTick = clamp(targetTick, minTick, maxTick);

            controllerauto.setGoal(new KineticState(targetTick));

            double power = controllerauto.calculate(
                    new KineticState(currentTick, motor.getVelocity())
            );

            motor.setPower(clamp(power, -1.0, 1.0));
            return;
        }

        if (snapModeEnabled) {

            double currentTick = motor.getCurrentPosition() - resetTick;
            turretAngle = encoderTicksToAngle(currentTick);

            double error = Math.IEEEremainder(snapTargetAngle - turretAngle, 360.0);

            if (!snapBrakeActive && Math.abs(error) < snapTolerance) {
                snapBrakeActive = true;
                snapBrakeAngle  = turretAngle;
                motor.brakeMode();
            }

            if (snapBrakeActive) {
                double brakeError = Math.IEEEremainder(snapBrakeAngle - turretAngle, 360.0);
                double targetTick = currentTick + angleToEncoderTicks(brakeError);

                controllerauto.setGoal(new KineticState(targetTick));

                double power = controllerauto.calculate(
                        new KineticState(currentTick, motor.getVelocity())
                );

                motor.setPower(clamp(power, -1.0, 1.0));
                return;
            }

            double targetTick = currentTick + angleToEncoderTicks(error);

            double maxTick = angleToEncoderTicks(max);
            double minTick = angleToEncoderTicks(min);

            if (targetTick > maxTick) {
                targetTick -= angleToEncoderTicks(360.0);
            } else if (targetTick < minTick) {
                targetTick += angleToEncoderTicks(360.0);
            }

            targetTick = clamp(targetTick, minTick, maxTick);

            controllerauto.setGoal(new KineticState(targetTick));

            double power = controllerauto.calculate(
                    new KineticState(currentTick, motor.getVelocity())
            );

            motor.setPower(clamp(power, -1.0, 1.0));
            return;
        }

        if (manualMode) {
            motor.setPower(manualDirection * manualPower);
            return;
        }

        double currentTick = motor.getCurrentPosition() - resetTick;

        turretAngle = encoderTicksToAngle(currentTick);

        toTurn = aimToObject(turretAngle);

        double targetTick = currentTick + angleToEncoderTicks(toTurn);

        double maxTick = angleToEncoderTicks(max);
        double minTick = angleToEncoderTicks(min);

        if (targetTick > maxTick) {
            targetTick -= angleToEncoderTicks(360.0);
        } else if (targetTick < minTick) {
            targetTick += angleToEncoderTicks(360.0);
        }

        targetTick = clamp(targetTick, minTick, maxTick);

        controllerauto.setGoal(new KineticState(targetTick));

        double power = controllerauto.calculate(
                new KineticState(
                        currentTick,
                        motor.getVelocity()
                )
        );


        motor.setPower(clamp(power, -1.0, 1.0));
    }

    private static double angleToEncoderTicks(double degrees) {
        return degrees / AutoConstants.Calculos.scalingFactor;
    }
}
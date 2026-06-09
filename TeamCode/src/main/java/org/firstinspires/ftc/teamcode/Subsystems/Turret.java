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

    public static double Tkp  = 0.00003;
    public static double Tki  = 0.00000000001;
    public static double Tkd  = 0.0005;
    public static double offsetx  = -9;

    private double robotX     = 0.0;
    private double robotY     = 0.0;
    private double headingRad = 0.0;
    private double compensation = 0.0;

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
    public static double headingOffset = -12;
    public static boolean followEnabled    = true;
    public static boolean headingEnabled   = true;
    public static boolean lockAngleEnabled = false;
    public static double  lockedAngle      = 0.0;
    private static boolean shootLockEnabled = false;
    private static double shootLockAngle = 0.0;
    private double lockedHeadingRad = 0.0;
    private double resetTick = 0.0;
    private static final MotorEx motor = new MotorEx("turret");

    private static final double max =  90;
    private static final double min = -270;
    private Turret() {}

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

    @Override
    public void initialize() {
        controllerauto = ControlSystem.builder()
                .posPid(Tkp, Tki, Tkd)
                .build();


        headingEnabled   = true;
        manualMode       = false;
        lockAngleEnabled = false;
        followEnabled    = true;
        resetTick        = 0.0;
    }

    public void reset() {
        motor.zeroed();
        resetTick        = 0.0;
        headingEnabled   = true;
        manualMode       = false;
        lockAngleEnabled = false;
        followEnabled    = true;
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

            double projectedAngle = currentTurretAngle + error;
            if (projectedAngle > max) {
                error = max - currentTurretAngle;
            } else if (projectedAngle < min) {
                error = min - currentTurretAngle;
            }

            return error;
        }

        calculatedDestinationAngle = Math.toDegrees(
                Math.atan2(goaly - robotY, goalx - robotX));


        double dx = goalx - robotX;
        double dy = goaly - robotY;
        double dist = Math.sqrt(dx * dx + dy * dy);


//        double leadAngleDeg = 0.0;
//        if (dist > 1.0) {
//            leadAngleDeg = Math.toDegrees(Math.atan2(compensation, dist));
//        }

        //cos 180 = -1
        //sen 90 = 1
        //sen 180 = 0


        //double robotAngleDeg = Math.toDegrees(headingRad);
        // double offsetxturret = offsetx * Math.cos(robotAngleDeg);
        //destinationAngle = calculatedDestinationAngle + compensation + RecoveryOffset + offsetxturret;

        double offsetxturret = offsetx * Math.sin(headingRad);
        double robotAngleDeg = Math.toDegrees(headingRad);
        destinationAngle = calculatedDestinationAngle + compensation + RecoveryOffset + offsetxturret;

        double error = destinationAngle - (robotAngleDeg + headingOffset) - currentTurretAngle;
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

        if (shootLockEnabled) {
            double currentTick = motor.getCurrentPosition() - resetTick;

            turretAngle = encoderTicksToAngle(currentTick);

            double error = shootLockAngle - turretAngle;
            error = Math.IEEEremainder(error, 360.0);

            double targetTick = currentTick + angleToEncoderTicks(error);

            targetTick = clamp(
                    targetTick,
                    angleToEncoderTicks(min),
                    angleToEncoderTicks(max)
            );

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

        targetTick = clamp(
                targetTick,
                angleToEncoderTicks(min),
                angleToEncoderTicks(max)
        );

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

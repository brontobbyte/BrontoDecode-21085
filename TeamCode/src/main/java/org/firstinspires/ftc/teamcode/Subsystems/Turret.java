package org.firstinspires.ftc.teamcode.Subsystems;

import static com.pedropathing.math.MathFunctions.clamp;
import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.Calculos.encoderTicksToAngle;
import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.Calculos.turnTurretBy;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.AutoPoses;
import org.opencv.core.Mat;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.positionable.SetPosition;

@Configurable
public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();
    public static ControlSystem controllerauto;
    public static double Tkp = 0.012;
    public static double Tki = 0;
    public static double Tkd = 0.001;
    private double robotY;
    private double robotX;
    public static double angleLL;
    private double realAngleLL = 0;
    public static double destinationAngle;
    private double lastValidDestinationAngle = 0.0;
    private boolean hasValidLastAngle = false;
    private static double heading;
    public static double toTurn;
    public static double div = 0;

    public static double goalx = 132;
    public static double goaly = 137;
    public static double multiMax = 5;
    public static double alpha = 0.1;
    public static double filteredVision = 0.1;

    public static double turretAngle;
    public static double calculatedDestinationAngle;

    public static double offset = 3;
    public static double RecoveryOffset = 0;

    private static double visionMultiplier = 0.380;
    private static double offsetAdjustmentRate = 0.43;
    private static boolean azul = true;
    private boolean wrapped = false;
    public int contador = 0;
    Telemetry telemetry;

    public static boolean manualMode = false;
    public static double manualDirection = 0;
    public static double manualPower = 0.6;
    public static boolean lockAngleEnabled = false;
    public static double lockedAngle = 0;
    public static double realAngleLLcorrected = 0;
    public static boolean followEnabled = true;
    public static boolean headingEnabled = true;


    private Turret() {
    }

    public void setPoseTracker(double robotX, double robotY, double heading, double angleLL, boolean azul, Telemetry telemetry) {
        this.robotX = robotX;
        this.robotY = robotY;
        this.heading = heading;
        Turret.angleLL = angleLL;
        this.telemetry = telemetry;
        if (azul) {
            goalx = AutoPoses.goalPoseazul.getX();
            goaly = AutoPoses.goalPoseazul.getY();
        } else {
            goalx = AutoPoses.goalPoseVermelho.getX();
            goaly = AutoPoses.goalPoseVermelho.getY();
        }
    }

    private static final MotorEx motor = new MotorEx("turret");

    public static void addOffset(double angle) {
        offset += angle;
    }

    public static void addRecOffset() {
        RecoveryOffset = 0.3;
    }

    public static void lessRecOffset() {
        RecoveryOffset = -0.3;
    }

    public static void stopRecOffset() {
        RecoveryOffset = 0;
    }

    public static void setManualMode(boolean enabled, double direction) {
        manualMode = enabled;
        manualDirection = direction;
        if (enabled) {
            followEnabled = false;
        }
    }

    public static void lockCurrentAngle() {
        turretAngle = encoderTicksToAngle(motor.getRawTicks());
        lockedAngle = turretAngle + heading;
        lockAngleEnabled = true;
        followEnabled = true;
    }
    public static void unlockAngle() {
        lockAngleEnabled = false;
    }
    @Override
    public void initialize() {
    }
    public void reset() {
        headingEnabled = true;
        motor.zeroed();
        hasValidLastAngle = false;
        lastValidDestinationAngle = 0.0;
        wrapped = false;
        manualMode = false;
        lockAngleEnabled = false;
        followEnabled = true;
    }

    public static void disableHeading() {
        headingEnabled = false;
    }

    public static void enableHeading() {
        headingEnabled = true;
    }

    public double aimToObject() {
        if (manualMode) {
            return manualDirection * 50;
        }

        double robotYPosition = robotY;
        double robotXPosition = robotX;

        calculatedDestinationAngle = Math.toDegrees(Math.atan2(goaly - robotYPosition, goalx - robotXPosition));

        boolean limelightActive = Math.abs(angleLL) > 0.3;

        filteredVision = alpha * angleLL + (1 - alpha) * filteredVision;

        /*
        if (lockAngleEnabled) {
            destinationAngle = lockedAngle;
        } else if (followEnabled && limelightActive) {
            if (Math.abs(angleLL) > 2) {
                offset += -Math.signum(angleLL) * offsetAdjustmentRate;
            }

            realAngleLL = filteredVision * visionMultiplier + offset;

            destinationAngle = calculatedDestinationAngle + realAngleLL;

            lastValidDestinationAngle = destinationAngle;
            hasValidLastAngle = true;
        } else if (followEnabled) {
            if (hasValidLastAngle) {
                destinationAngle = lastValidDestinationAngle;
            } else {
                destinationAngle = calculatedDestinationAngle;
            }
        } else {
            destinationAngle = calculatedDestinationAngle;
        }


         */
        contador++;
        if (contador%20 == 0 && Math.abs(angleLL) > 4 && Math.abs(motor.getVelocity()) < 300){
            realAngleLLcorrected += angleLL;
            contador = 0;
        }
        if (Math.abs(motor.getVelocity()) < 300 && Math.abs(angleLL) > 4){
            destinationAngle = calculatedDestinationAngle - angleLL/2 - realAngleLLcorrected/2;
            realAngleLL = angleLL/2;
        }else{
            destinationAngle = calculatedDestinationAngle - realAngleLL - realAngleLLcorrected/2;
        }
        if (Math.abs(angleLL) < 4){
            realAngleLLcorrected = 0;
        }


        turretAngle = encoderTicksToAngle(motor.getRawTicks());
        double robotAngle = headingEnabled ? heading : 0;
        turretAngle = ((turretAngle + 170) % 360) - 170;
        destinationAngle = ((destinationAngle + 170) % 360) - 170;

        toTurn = destinationAngle - (turretAngle + robotAngle);
        toTurn = ((toTurn + 180) % 360) - 180;

        if (turretAngle > 170 || (toTurn > 0 && turretAngle + toTurn > 170)) {
            toTurn -= 360;
            wrapped = true;
        } else if (turretAngle < -170 || (toTurn < 0 && turretAngle + toTurn < -170)) {
            toTurn += 360;
            wrapped = true;
        } else {
            wrapped = false;
        }

        return toTurn;
    }

    @Override
    public void periodic() {
        controllerauto = ControlSystem.builder()
                .posPid(Tkp, Tki, Tkd)
                .build();
        if (manualMode) {
            motor.setPower(manualDirection * manualPower);
            return;
        }

        double power = turnTurretBy(aimToObject(), angleLL, div);
        if (Math.abs(toTurn) < 1) {
            power = 0;
        }
        motor.setPower(power + RecoveryOffset);
    }
}
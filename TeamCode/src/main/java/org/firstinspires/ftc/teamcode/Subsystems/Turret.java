package org.firstinspires.ftc.teamcode.Subsystems;

import static com.pedropathing.math.MathFunctions.clamp;
import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.Calculos.encoderTicksToAngle;
import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.Calculos.turnTurretBy;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.positionable.SetPosition;
 //bababa
@Configurable
public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();
    private double robotY;
    private double robotX;
    public static double angleLL;
    private double realAngleLL = 0;
    public static double destinationAngle;
    private double lastValidDestinationAngle = 0.0;
    private boolean hasValidLastAngle = false;
    private double heading;
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

    private Turret() {
    }
    public void setPoseTracker(double robotX, double robotY, double heading, double angleLL, boolean azul, Telemetry telemetry) {
        this.robotX = robotX;
        this.robotY = robotY;
        this.heading = heading;
        Turret.angleLL = angleLL;
        this.telemetry = telemetry;
        if (azul){
            goalx = 10;
            goaly = 137;
        }else {
            goalx = 124;
            goaly = 137;
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


    @Override
    public void initialize() {
    }
    public void reset() {
        motor.zeroed();
        hasValidLastAngle = false;
        lastValidDestinationAngle = 0.0;
        wrapped = false;
    }
    public double aimToObject() {
        double robotYPosition = robotY, robotXPosition = robotX;

        calculatedDestinationAngle = Math.toDegrees(Math.atan2(goaly - robotYPosition,robotXPosition - goalx));
        boolean limelightActive = Math.abs(angleLL) > 0.3;
        /*
        if (limelightActive) {
            if (Math.abs(angleLL) > 2) {
                offset += -Math.signum(angleLL) * offsetAdjustmentRate;
            }
            realAngleLL = angleLL * visionMultiplier + offset;
            destinationAngle = calculatedDestinationAngle + realAngleLL;

            lastValidDestinationAngle = destinationAngle;
            hasValidLastAngle = true;
        } else {
            if (hasValidLastAngle) {
                destinationAngle = lastValidDestinationAngle;
            } else {
                destinationAngle = calculatedDestinationAngle;
            }
        }
         */
        filteredVision = alpha * angleLL + (1 - alpha) * filteredVision;
        if (Math.abs(angleLL) > 2 && Math.abs(angleLL) < multiMax){
            //angleLL = angleLL * multi;
        }
        contador++;
        if (Math.abs(motor.getVelocity()) < 50 && Math.abs(angleLL) > 4 && (contador%50 == 0)){
            realAngleLL = angleLL;
            contador = 0;
            //telemetry.addData("errado", "errado");
            //telemetry.update();
        }else{
            div = 0;
        }
        destinationAngle = calculatedDestinationAngle - filteredVision - realAngleLL - offset;
        turretAngle = encoderTicksToAngle(motor.getRawTicks());
        double robotAngle = heading;

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
        double power = turnTurretBy(aimToObject(), angleLL, div);
        if (Math.abs(toTurn) < 1) {
            power = 0;
        }
        motor.setPower(power + RecoveryOffset);
    }
}
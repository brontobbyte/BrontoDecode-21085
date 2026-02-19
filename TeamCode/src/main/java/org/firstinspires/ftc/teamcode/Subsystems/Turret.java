package org.firstinspires.ftc.teamcode.Subsystems;

import static com.pedropathing.math.MathFunctions.clamp;
import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.Calculos.encoderTicksToAngle;
import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.Calculos.turnTurretBy;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();
    private double robotY;
    private double robotX;
    public static double angleLL;
    private double realAngleLL;
    public static double destinationAngle;
    private double lastValidDestinationAngle = 0.0;
    private boolean hasValidLastAngle = false;
    private double heading;
    public static double toTurn;
    public static double goalx = 132;
    public static double goaly = 137;
    public static double multiMax = 5;
    public static double multi = 1.5;

    public static double turretAngle;
    public static double calculatedDestinationAngle;

    public static double offset = 0;
    private static double visionMultiplier = 0.380;
    private static double offsetAdjustmentRate = 0.43;
    private static boolean azul = true;
    private boolean wrapped = false;

    private Turret() {
    }
    public void setPoseTracker(double robotX, double robotY, double heading, double angleLL, boolean azul) {
        this.robotX = robotX;
        this.robotY = robotY;
        this.heading = heading;
        Turret.angleLL = angleLL;
        if (azul){
            goalx = 10;
            goaly = 137;
        }else {
            goalx = 124;
            goaly = 137;
        }
    }
    private static final MotorEx motor = new MotorEx("turret");

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

        calculatedDestinationAngle = Math.toDegrees(Math.atan2(robotYPosition - goaly, goalx - robotXPosition));
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
        if (Math.abs(angleLL) > 2 && Math.abs(angleLL) < multiMax){
            angleLL = angleLL * multi;
        }
       destinationAngle = calculatedDestinationAngle - (clamp(angleLL, -10, 10));
        turretAngle = encoderTicksToAngle(motor.getRawTicks());
      double robotAngle = -heading;

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
        double power = turnTurretBy(aimToObject(), angleLL);
        if (Math.abs(toTurn) < 1) {
            power = 0;
        }
        motor.setPower(power);
    }
}
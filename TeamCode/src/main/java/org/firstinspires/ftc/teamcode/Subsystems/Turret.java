package org.firstinspires.ftc.teamcode.Subsystems;

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
    private double angleLL;
    private double realAngleLL;

    public static double destinationAngle;
    public static double destinationAngleLL;

    private double heading;
    public static double toTurn;
    public static double goalx = 10;
    public static double goaly = 137;
    public static double turretAngle;
    public static int contador;

    private Turret() {
    }
    public void setPoseTracker(double robotX, double robotY, double heading, double angleLL) {
        this.robotX  =  robotX;
        this.robotY  =  robotY;
        this.heading = heading;
        this.angleLL = angleLL;
    }
    private static final MotorEx motor = new MotorEx("turret");

    @Override
    public void initialize() {
    }
    public void reset() {
        motor.zeroed();
    }
    public double aimToObject(){
        double robotYPosition = robotY, robotXPosition = robotX;
        destinationAngle = Math.toDegrees(Math.atan2(goaly - robotYPosition,
                goalx - robotXPosition));
        contador ++;
        if ((contador % 100 == 0) && (angleLL != 0.0)) {
            realAngleLL = angleLL;
            contador = 0;
        }
        destinationAngleLL = destinationAngle + realAngleLL;
        turretAngle = encoderTicksToAngle(motor.getRawTicks());
        double robotAngle = heading;
        toTurn = destinationAngleLL - (turretAngle + robotAngle);
        return (toTurn);
    }

    @Override
    public void periodic() {
        motor.setPower((turnTurretBy(aimToObject())));
    }
}
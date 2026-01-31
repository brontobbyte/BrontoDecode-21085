package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.Calculos.angleToEncoderTicks;
import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.Calculos.encoderTicksToAngle;
import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.Calculos.turnTurretBy;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.PoseTracker;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.control.ControlSystem;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
@Configurable
public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();
    private static Localizer localizer;
    public HardwareMap hardwareMap;
    double protecao;
    private double robotY;
    private double robotX;
    private double angleLL;
    public static double destinationAngle;
    public static double destinationAngleLL;

    private double heading;
    public static double toTurn;
    public static double goalx = 10;
    public static double goaly = 137;
    public static double turretAngle;
    private PoseTracker poseTracker;
    private Turret() {
    }
    public void setPoseTracker(double robotX, double robotY, double heading, double angleLL) {
        this.robotX  =  robotX;
        this.robotY  =  robotY;
        this.heading = heading;
        this.angleLL = angleLL;
    }
    private static MotorEx motor = new MotorEx("turret");
    private final double ticks360 = 1000.0;
    private boolean limelightTracking = false;
    private double targetDegrees = 0;

    @Override
    public void initialize() {
    }

    public MotorEx getMotor() {
        return motor;
    }

    public double aimToObject(){
        double robotYPosition = robotY, robotXPosition = robotX;
        destinationAngle = Math.toDegrees(Math.atan2(goaly - robotYPosition,
                goalx - robotXPosition));
        //destinationAngleLL = (turretAngle-robotAngle) + angleLL;
        destinationAngleLL = destinationAngle + angleLL;
        turretAngle = encoderTicksToAngle(motor.getRawTicks());
        double robotAngle = heading;
        toTurn = destinationAngleLL - (turretAngle + robotAngle);
        return (toTurn);
        // take the mod/remainder of toTurn/360
        // to keep the angle in the range of [0,360]
    }

    @Override
    public void periodic() {
        motor.setPower((turnTurretBy(aimToObject())));
    }
}
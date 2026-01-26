package org.firstinspires.ftc.teamcode.Subsystems;
import static com.rowanmcalpin.nextftc.ftc.OpModeData.hardwareMap;

import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.Calculos.encoderTicksToAngle;
import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.Calculos.turnTurretBy;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.PoseTracker;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants.AutoConstants;

import java.util.List;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.AngleType;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();
    private static Localizer localizer;
    private double robotY;
    private double robotX;
    private PoseTracker poseTracker;
    private Turret() {
    }
    public void setPoseTracker(PoseTracker poseTracker) {
        this.poseTracker = poseTracker;
    }
    private static MotorEx motor = new MotorEx("motor_turret");
    private Limelight3A limelight;
    private static IMU imu;
    private final double ticks360 = 1000.0;
    private boolean limelightTracking = false;
    private final ControlSystem controlSystem = ControlSystem.builder()
            .angular(AngleType.DEGREES, feedback -> feedback.posPid(0.043, 0.000, 0.0))
            .build();

    private double targetDegrees = 0;

    public void initialize(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(4);
        limelight.start();
    }
    public MotorEx getMotor() {
        return motor;
    }
    public double aimToObject(){
        double robotYPosition = robotY, robotXPosition = robotX;
        double destinationAngle = Math.atan2(132 - robotYPosition,
                137 - robotXPosition);

        double turretAngle = encoderTicksToAngle(motor.getRawTicks());
        double robotAngle = imu.getRobotYawPitchRollAngles().getYaw();

        double toTurn = destinationAngle - (turretAngle + robotAngle);
        return (toTurn%360);
         // take the mod/remainder of toTurn/360
        // to keep the angle in the range of [0,360]
    }

    @Override
    public void periodic() {
        Pose pose = poseTracker.getPose();

        double robotX = pose.getX();
        double robotY = pose.getY();
        double heading = pose.getHeading();

        motor.setPower(turnTurretBy(aimToObject()));

    }
}
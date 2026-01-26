package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.Calculos.encoderTicksToAngle;
import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.Calculos.turnTurretBy;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.PoseTracker;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.feedback.AngleType;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();

    private Turret() {}

    private static MotorEx motor = new MotorEx("motor_turret");
    private static IMU imu;

    private static PoseTracker poseTracker = null;

    public static double Tkp = 0;
    public static double Tki = 0;
    public static double Tkd = 0;

    private final ControlSystem controlSystem = ControlSystem.builder()
            .angular(AngleType.DEGREES, feedback -> feedback.posPid(Tkp, Tki, Tkd))
            .build();

    public void init(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        ));
    }

    public MotorEx getMotor() {
        return motor;
    }

    public void setPoseTracker(Localizer localizer) {
        if (localizer != null) {
            poseTracker = localizer.getPoseTracker();
        }
    }
    public void setPoseTracker(PoseTracker tracker) {
        poseTracker = tracker;
    }

    private static double angleDifference(double target, double current) {
        double diff = target - current;
        while (diff > 180) diff -= 360;
        while (diff < -180) diff += 360;
        return diff;
    }

    public double aimToObject() {
        double robotYPosition = 0;
        double robotXPosition = 0;

        if (poseTracker != null) {
            Pose currentPose = poseTracker.getPose();
            if (currentPose != null) {
                robotXPosition = currentPose.x;
                robotYPosition = currentPose.y;
            }
        }

        double destinationAngleRad = Math.atan2(132 - robotYPosition, 137 - robotXPosition);
        double destinationAngle = Math.toDegrees(destinationAngleRad);

        double turretAngle = encoderTicksToAngle(motor.getRawTicks());
        double robotAngle = imu.getRobotYawPitchRollAngles().getYaw();

        double targetAngle = destinationAngle - robotAngle;

        double toTurn = angleDifference(targetAngle, turretAngle);

        if ((turretAngle >= 180 && toTurn > 0) || (turretAngle <= -180 && toTurn < 0)) {
            toTurn = 0;
        }

        return toTurn;
    }

    public static Command TurretAlign = new LambdaCommand()
            .setStart(() -> {
            })
            .setUpdate(() -> {
                double robotYPosition = 0, robotXPosition = 0;

                if (poseTracker != null) {
                    Pose currentPose = poseTracker.getPoseEstimate();
                    if (currentPose != null) {
                        robotXPosition = currentPose.x;
                        robotYPosition = currentPose.y;
                    }
                }

                double destinationAngleRad = Math.atan2(132 - robotYPosition, 137 - robotXPosition);
                double destinationAngle = Math.toDegrees(destinationAngleRad);

                double turretAngle = encoderTicksToAngle(motor.getRawTicks());
                double robotAngle = imu.getRobotYawPitchRollAngles().getYaw();

                double currentTotalAngle = turretAngle + robotAngle;
                double toTurn = destinationAngle - currentTotalAngle;

                while (toTurn > 180) toTurn -= 360;
                while (toTurn < -180) toTurn += 360;

                turnTurretBy(toTurn);
            })
            .setStop(interrupted -> {})
            .setIsDone(() -> false)
            .setInterruptible(false)
            .named("Turret Align");

    @Override
    public void periodic() {
        double power = turnTurretBy(aimToObject());
        motor.setPower(power);
    }
}
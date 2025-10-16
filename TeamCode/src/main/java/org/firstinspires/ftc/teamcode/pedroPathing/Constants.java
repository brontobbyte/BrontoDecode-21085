package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-33.5851582052308)
            .lateralZeroPowerAcceleration(-85.20701687500193)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.3, 0.0001, 0.015, 0.0001))
            .headingPIDFCoefficients(new PIDFCoefficients(1.8, 0.1, 0.1, 0.01));

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardTicksToInches(0.003043250375564735)
            .strafeTicksToInches(0.0034150566392557926)
            .forwardEncoder_HardwareMapName("odometria")
            .strafeEncoder_HardwareMapName("motor_esquerda")
            .forwardPodY(-4.5)
            .strafePodX(-8)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                    )
            );

    public static MecanumConstants driveConstants = new MecanumConstants()
            .xVelocity(56.05057964)
            .yVelocity(37.461833)
            .maxPower(1)
            .rightFrontMotorName("motor_direita")
            .rightRearMotorName("motor_direitatras")
            .leftRearMotorName("motor_esquerdatras")
            .leftFrontMotorName("motor_esquerda")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .twoWheelLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}

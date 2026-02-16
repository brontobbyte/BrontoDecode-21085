package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .centripetalScaling(0.0005)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.003, 0.001, 0.0008,0.001,0.0001))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0.0001, 0.001, 0.0001))
            //.secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(1.2, 0.001, 0.05, 0.001,0.01))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.09, 0.0003, 0.008, 0.03))
            //.secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1, 0.0001, 0.01, 0.003))
            .lateralZeroPowerAcceleration(-57.482201)
            .forwardZeroPowerAcceleration(-50.8618072)
            .mass(11.840);


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(76.9793)
            .yVelocity(42.2676756);

    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(0.0030029318414923315)
            .strafeTicksToInches(0.0029361879034900417)
            .turnTicksToInches(0.0028128936741553416)
            .leftPodY(4)
            .rightPodY(-4)
            .strafePodX(-8)
            .leftEncoder_HardwareMapName("fl")
            .rightEncoder_HardwareMapName("br")
            .strafeEncoder_HardwareMapName("f2")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelIMULocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}

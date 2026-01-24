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
            .centripetalScaling(0.00005)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.005, 0.001, 0.001,0,0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0.007, 0.08, 0))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.14, 0.0007, 0.03, 0))
            .lateralZeroPowerAcceleration(-70.018816)
            .forwardZeroPowerAcceleration(-41.241598)
            .mass(9);


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("motor_direita")
            .rightRearMotorName("motor_direitatras")
            .leftRearMotorName("motor_esquerdatras")
            .leftFrontMotorName("motor_esquerda")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(60.166272)
            .yVelocity(40.58628468);

    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(.0030135413641320823)
            .strafeTicksToInches(.003381839207377)
            .turnTicksToInches(.0032552252158545945)
            .leftPodY(4)
            .rightPodY(-4)
            .strafePodX(-8)
            .leftEncoder_HardwareMapName("motor_shooter2")
            .rightEncoder_HardwareMapName("motor_direitatras")
            .strafeEncoder_HardwareMapName("motor_esquerda")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.REVERSE)
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

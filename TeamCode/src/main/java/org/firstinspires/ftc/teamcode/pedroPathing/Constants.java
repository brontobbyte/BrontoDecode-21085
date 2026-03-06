package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .centripetalScaling(0.0005)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.029, 0.001, 0.0008,0.001,0.0001))
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

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(5.3543307087)
            .strafePodX(-4.9)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

}

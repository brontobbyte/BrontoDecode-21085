package org.firstinspires.ftc.teamcode.Constants;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

@Configurable
public class AutoPathsVermelho {
    public static PathChain InicialIntake = follower.pathBuilder()
            .addPath(
                    new BezierCurve(
                            AutoPoses.poseInicial.mirror(),
                            AutoPoses.intakeCurvedPose.mirror(),
                            AutoPoses.intakePose.mirror()
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
            .build();

    public static PathChain Shoot1 = follower.pathBuilder()
            .addPath(
                    new BezierLine(
                            AutoPoses.intakePose.mirror(),
                            AutoPoses.shootPose1.mirror()
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();

    public static PathChain Gate = follower.pathBuilder()
            .addPath(
                    new BezierLine(
                            AutoPoses.shootPose1.mirror(),
                            AutoPoses.intakeGatePose.mirror()
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))
            .build();

    public static PathChain ShootDoGate = follower.pathBuilder()
            .addPath(
                    new BezierLine(
                            AutoPoses.shootPose2.mirror(),
                            AutoPoses.shootPose2.mirror()
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
            .build();

    public static PathChain Intake2 = follower.pathBuilder()
            .addPath(
                    new BezierCurve(
                            AutoPoses.shootPose2.mirror(),
                            AutoPoses.intake2CurvedPose.mirror(),
                            AutoPoses.intake2Pose.mirror()
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();

    public static PathChain Shoot2 = follower.pathBuilder()
            .addPath(
                    new BezierLine(
                            AutoPoses.intake2Pose.mirror(),
                            AutoPoses.shootPose1.mirror()
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();

    public static PathChain Intake3 = follower.pathBuilder()
            .addPath(
                    new BezierCurve(
                            AutoPoses.shootPose2.mirror(),
                            AutoPoses.intake3CurvedPose.mirror(),
                            AutoPoses.intake3Pose.mirror()
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();

    public static PathChain Shoot3 = follower.pathBuilder()
            .addPath(
                    new BezierLine(
                            AutoPoses.intake3Pose.mirror(),
                            AutoPoses.shootPose1.mirror()
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();

    public static PathChain preintake = follower.pathBuilder()
            .addPath(
                    new BezierCurve(
                            AutoPoses.intakePose.mirror(),
                            new Pose(60, 70).mirror(),
                            new Pose(60, 70).mirror()
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();

    public static PathChain AbrirGate = follower.pathBuilder()
            .addPath(
                    new BezierCurve(
                            new Pose(29.51, 65).mirror(),
                            new Pose(2, 65).mirror(),
                            new Pose(29, 65).mirror()
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();

    public static PathChain Intake4 = follower.pathBuilder()
            .addPath(
                    new BezierCurve(
                            new Pose(49, 35).mirror(),
                            new Pose(9, 35).mirror()
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();

    public static PathChain Shoot4 = follower.pathBuilder()
            .addPath(
                    new BezierLine(
                            new Pose(15, 35).mirror(),
                            AutoPoses.intakePose.mirror()
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();

    public static PathChain preintake2 = follower.pathBuilder()
            .addPath(
                    new BezierLine(
                            AutoPoses.intakePose.mirror(),
                            new Pose(49, 35).mirror()
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();

    public static PathChain Intake5 = follower.pathBuilder()
            .addPath(
                    new BezierCurve(
                            new Pose(30, 45).mirror(),
                            new Pose(10, 45).mirror(),
                            new Pose(10, 0).mirror()
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
            .build();
}
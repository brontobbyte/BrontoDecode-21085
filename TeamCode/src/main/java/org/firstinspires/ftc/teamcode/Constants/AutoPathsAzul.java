package org.firstinspires.ftc.teamcode.Constants;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

@Configurable
public class AutoPathsAzul {
    public static PathChain InicialIntake = follower.pathBuilder()
            .addPath(
                    new BezierCurve(
                            AutoPoses.poseInicial,
                            AutoPoses.intakeCurvedPose,
                            AutoPoses.intakePose
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
            .build();

    public static PathChain Shoot1 = follower.pathBuilder()
            .addPath(
                    new BezierLine(
                            AutoPoses.intakePose,
                            AutoPoses.shootPose1
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();

    public static PathChain Gate = follower.pathBuilder()
            .addPath(
                    new BezierLine(
                            AutoPoses.shootPose1,
                            AutoPoses.intakeGatePose
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))
            .build();

    public static PathChain ShootDoGate = follower.pathBuilder()
            .addPath(
                    new BezierLine(
                            AutoPoses.shootPose2,
                            AutoPoses.shootPose2
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
            .build();

    public static PathChain Intake2 = follower.pathBuilder()
            .addPath(
                    new BezierCurve(
                            AutoPoses.shootPose2,
                            AutoPoses.intake2CurvedPose,
                            AutoPoses.intake2Pose
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();

    public static PathChain Shoot2 = follower.pathBuilder()
            .addPath(
                    new BezierLine(
                            AutoPoses.intake2Pose,
                            AutoPoses.shootPose1
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();

    public static PathChain Intake3 = follower.pathBuilder()
            .addPath(
                    new BezierCurve(
                            AutoPoses.shootPose2,
                            AutoPoses.intake3CurvedPose,
                            AutoPoses.intake3Pose
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();

    public static PathChain Shoot3 = follower.pathBuilder()
            .addPath(
                    new BezierLine(
                            AutoPoses.intake3Pose,
                            AutoPoses.shootPose1
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();

    public static PathChain preintake = follower.pathBuilder()
            .addPath(
                    new BezierCurve(
                            AutoPoses.intakePose,
                            new Pose(60, 70)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();

    public static PathChain AbrirGate = follower.pathBuilder()
            .addPath(
                    new BezierCurve(
                            new Pose(29.51, 65),
                            new Pose(2, 65),
                            new Pose(29, 65)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();

    public static PathChain Intake4 = follower.pathBuilder()
            .addPath(
                    new BezierCurve(
                            new Pose(49, 35),
                            new Pose(9, 35)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();

    public static PathChain Shoot4 = follower.pathBuilder()
            .addPath(
                    new BezierLine(
                            new Pose(15, 35),
                            AutoPoses.intakePose
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();

    public static PathChain preintake2 = follower.pathBuilder()
            .addPath(
                    new BezierLine(
                            AutoPoses.intakePose,
                            new Pose(49, 35)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();

    public static PathChain Intake5 = follower.pathBuilder()
            .addPath(
                    new BezierCurve(
                            new Pose(30, 45),
                            new Pose(10, 45),
                            new Pose(10, 0)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
            .build();
}
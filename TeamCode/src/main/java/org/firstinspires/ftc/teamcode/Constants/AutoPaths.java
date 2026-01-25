package org.firstinspires.ftc.teamcode.Constants;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

@Configurable
public class AutoPaths {
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
            .setVelocityConstraint(0.5)
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
}
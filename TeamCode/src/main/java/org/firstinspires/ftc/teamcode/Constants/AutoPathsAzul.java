package org.firstinspires.ftc.teamcode.Constants;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

@Configurable
public class AutoPathsAzul {

    public static double gateAngle = 155;
    public static PathChain ShootPreload(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.poseInicial,
                                AutoPoses.shootPose1
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public static PathChain IntakeMeio(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                AutoPoses.shootPose1,
                                AutoPoses.intakeMeioCurvedPose,
                                AutoPoses.intakeMeioPose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public static PathChain ShootMeio(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.intakeMeioPose,
                                AutoPoses.shootPose1
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public static PathChain Gate(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                AutoPoses.shootPose1,
                                AutoPoses.gateCurvedPose,
                                AutoPoses.gatePose2
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(gateAngle))
                .build();
    }
    public static PathChain Gate2(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.gatePose2,
                                AutoPoses.gatePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(gateAngle), Math.toRadians(gateAngle))
                .build();
    }

    public static PathChain GateCicle(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.gatePose,
                                AutoPoses.gateCiclePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(150))
                .build();
    }
    public static PathChain GateCicleFinal(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.gateCiclePose,
                                AutoPoses.gateCiclePoseFinal
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                .build();
    }

    public static PathChain ShootGate(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.gateCiclePose,
                                AutoPoses.shootPose1
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(180))
                .build();
    }

    public static PathChain ShootGateCima(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.gateCiclePose,
                                AutoPoses.shootPose1
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                .build();
    }

    public static PathChain IntakeCima(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                AutoPoses.shootPose1,
                                AutoPoses.intakeCimaCurvedPose,
                                AutoPoses.intakeCimaPose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public static PathChain ShootCima(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.intakeCimaPose,
                                AutoPoses.shootPose1
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public static PathChain IntakeBaixo(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                AutoPoses.shootPose1,
                                AutoPoses.intakeBaixoCurvedPose,
                                AutoPoses.intakeBaixoPose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-130), Math.toRadians(180))
                .build();
    }

    public static PathChain shootPoselast(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.intakeBaixoPose,
                                AutoPoses.shootPose1
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-132), Math.toRadians(180))
                .build();
    }
    public static PathChain last (Follower follower){
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.shootPoselast,
                                AutoPoses.last
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();
    }
}
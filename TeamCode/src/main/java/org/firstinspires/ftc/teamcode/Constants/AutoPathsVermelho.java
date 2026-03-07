package org.firstinspires.ftc.teamcode.Constants;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

@Configurable
public class AutoPathsVermelho {
    public static PathChain ShootPreload(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.poseInicial.mirror(),
                                AutoPoses.shootPose1.mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public static PathChain IntakeMeio(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                AutoPoses.shootPose1.mirror(),
                                AutoPoses.intakeMeioCurvedPose.mirror(),
                                AutoPoses.intakeMeioPose.mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public static PathChain ShootMeio(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.gatePose.mirror(),
                                AutoPoses.shootPose1.mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public static PathChain Gate(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                AutoPoses.shootPose1.mirror(),
                                AutoPoses.gateCurvedPose.mirror(),
                                AutoPoses.gatePose.mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(20))
                .build();
    }
    public static PathChain Gate2(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.intakeMeioPose.mirror(),
                                AutoPoses.gatePose2.mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
                .build();
    }

    public static PathChain GateCicle(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.gatePose.mirror(),
                                AutoPoses.gateCiclePose.mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
                .build();
    }
    public static PathChain GateCicleFinal(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.gateCiclePose.mirror(),
                                AutoPoses.gateCiclePoseFinal.mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(90))
                .build();
    }

    public static PathChain ShootGate(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.gateCiclePose.mirror(),
                                AutoPoses.shootPose1.mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public static PathChain ShootGateCima(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.gateCiclePose.mirror(),
                                AutoPoses.shootPose1.mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))
                .build();
    }

    public static PathChain IntakeCima(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                AutoPoses.shootPose1.mirror(),
                                AutoPoses.intakeCimaCurvedPose.mirror(),
                                AutoPoses.intakeCimaPose.mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public static PathChain ShootCima(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.intakeCimaPose.mirror(),
                                AutoPoses.shootPose1.mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public static PathChain IntakeBaixo(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                AutoPoses.shootPose1.mirror(),
                                AutoPoses.intakeBaixoCurvedPose.mirror(),
                                AutoPoses.intakeBaixoPose.mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public static PathChain shootPoselast(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.intakeBaixoPose.mirror(),
                                AutoPoses.shootPose1.mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain last (Follower follower){
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.shootPoselast.mirror(),
                                AutoPoses.last.mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();
    }
}
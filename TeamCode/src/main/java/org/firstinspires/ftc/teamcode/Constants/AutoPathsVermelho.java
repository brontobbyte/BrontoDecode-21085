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
public class AutoPathsVermelho {
    public static PathChain ShootPreload(Follower follower){
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                AutoPoses.poseInicial.mirror(),
                                AutoPoses.shootPose1.mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();
    }
    public static PathChain IntakeMeio(Follower follower){
        return  follower.pathBuilder()
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
    public static PathChain ShootMeio(Follower follower){
        return  follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.intakeMeioPose.mirror(),
                                AutoPoses.shootPose1.mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain Gate(Follower follower){
        return  follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.shootPose1.mirror(),
                                AutoPoses.gatePose.mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain GateCicle(Follower follower){
        return  follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                AutoPoses.gatePose.mirror(),
                                AutoPoses.gateCicleCurvedPose.mirror(),
                                AutoPoses.gateCiclePose.mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();
    }
    public static PathChain ShootGate(Follower follower){
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                AutoPoses.gateCiclePose.mirror(),
                                AutoPoses.shootPose1.mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain ShootGateCima(Follower follower){
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                AutoPoses.gateCiclePose.mirror(),
                                AutoPoses.shootCurvedPose2.mirror(),
                                AutoPoses.shootPose2.mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain IntakeCima(Follower follower){
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                AutoPoses.shootPose2.mirror(),
                                AutoPoses.intakeCimaPose.mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain ShootCima(Follower follower){
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                AutoPoses.intakeCimaPose.mirror(),
                                AutoPoses.shootPose3.mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }


}
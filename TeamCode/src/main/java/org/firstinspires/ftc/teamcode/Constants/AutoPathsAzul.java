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
    public static PathChain ShootPreload(Follower follower){
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.poseInicial,
                                AutoPoses.shootPose1
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();
    }
    public static PathChain IntakeMeio(Follower follower){
        return  follower.pathBuilder()
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
    public static PathChain ShootMeio(Follower follower){
        return  follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.intakeMeioPose,
                                AutoPoses.shootPose1
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }
    public static PathChain Gate(Follower follower){
        return  follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.shootPose1,
                                AutoPoses.gatePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }
    public static PathChain GateCicle(Follower follower){
        return  follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                AutoPoses.gatePose,
                                AutoPoses.gateCicleCurvedPose,
                                AutoPoses.gateCiclePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();
    }
    public static PathChain ShootGate(Follower follower){
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.gateCiclePose,
                                AutoPoses.shootPose1
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }
    public static PathChain ShootGateCima(Follower follower){
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                AutoPoses.gateCiclePose,
                                AutoPoses.shootCurvedPose2,
                                AutoPoses.shootPose2
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }
    public static PathChain IntakeCima(Follower follower){
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.shootPose2,
                                AutoPoses.intakeCimaPose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }
    public static PathChain ShootCima(Follower follower){
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
    public static PathChain IntakeBaixo(Follower follower){
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                AutoPoses.shootPose1,
                                AutoPoses.intakeBaixoCurvedPose,
                                AutoPoses.intakeBaixoPose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }
    public static PathChain ShootBaixo(Follower follower){
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.intakeBaixoPose,
                                AutoPoses.shootPose3
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public static PathChain saida(Follower follower){
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.intakeBaixoPose,
                                AutoPoses.shootPose3
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }


}
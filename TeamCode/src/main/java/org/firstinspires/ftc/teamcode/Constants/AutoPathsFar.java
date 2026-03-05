package org.firstinspires.ftc.teamcode.Constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

@Configurable
public class AutoPathsFar {

    public static PathChain Intake(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPosesFar.PoseInicialFar,
                                AutoPosesFar.intakeEnd
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public static PathChain IntakeDuplicate(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPosesFar.intakeEnd,
                                AutoPosesFar.intakeDuplicateEnd
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public static PathChain IntakeDuplicate2(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPosesFar.intakeDuplicateEnd,
                                AutoPosesFar.intakeDuplicate2End
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public static PathChain ShootPoseFar(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPosesFar.intakeDuplicate2End,
                                AutoPosesFar.shootPoseFar
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public static PathChain FileiraBaixo(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                AutoPosesFar.shootPoseFar,
                                AutoPosesFar.fileiraBaixoControl,
                                AutoPosesFar.fileiraBaixoEnd
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public static PathChain ShootPoseFar1(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPosesFar.fileiraBaixoEnd,
                                AutoPosesFar.shootPoseFar_1
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public static PathChain Line7(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPosesFar.shootPoseFar_1,
                                AutoPosesFar.line7End
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    public static PathChain Line8(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPosesFar.line7End,
                                AutoPosesFar.line8End
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    public static PathChain Line9(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPosesFar.line8End,
                                AutoPosesFar.line9End
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    public static PathChain ShootPoseFar2(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPosesFar.line9End,
                                AutoPosesFar.shootPoseFar_2
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }
}
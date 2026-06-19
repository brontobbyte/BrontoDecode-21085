package org.firstinspires.ftc.teamcode.Constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

@Configurable
public class AutoPathsFar {

    public static PathChain IntakeHp(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPosesFar.shootPoseFar,
                                AutoPosesFar.intakeHp
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }
    public static PathChain IntakeHp2(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPosesFar.shootPoseFar,
                                AutoPosesFar.intakeHp2
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public static PathChain shootHp (Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPosesFar.intakeHp,
                                AutoPosesFar.shootPoseFar
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }
    public static PathChain shootHp2 (Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPosesFar.intakeHp,
                                AutoPosesFar.shootPoseFar
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public static PathChain intakeFileira(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                AutoPosesFar.PoseInicialFar,
                                AutoPosesFar.fileiraBaixoControl,
                                AutoPosesFar.fileiraBaixoEnd

                        )
                )

                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }
        public static PathChain shootintake1(Follower follower) {
            return follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    AutoPosesFar.fileiraBaixoEnd,
                                    AutoPosesFar.shootPoseFar
                            )
                    )

                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();


        }
}
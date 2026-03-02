package org.firstinspires.ftc.teamcode.Constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

@Configurable
public class AutoPathsFar {

    public static PathChain FileiraBaixo(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                AutoPosesFar.PoseInicialFar,
                                AutoPosesFar.fileiraBaixoControl,
                                AutoPosesFar.fileiraBaixoEnd
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();
    }

    public static PathChain ShootPose(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPosesFar.shoot2Start,
                                AutoPosesFar.shoot2End
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    public static PathChain IntakeHp(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                AutoPosesFar.intakeHpStart,
                                AutoPosesFar.intakeHpControl,
                                AutoPosesFar.intakeHpEnd
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
    }

    public static PathChain VoltaHp(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPosesFar.voltaHpStart,
                                AutoPosesFar.voltaHpEnd
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    public static PathChain Intake2(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPosesFar.intake2Start,
                                AutoPosesFar.intake2End
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
    }

    public static PathChain Path6(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPosesFar.path6Start,
                                AutoPosesFar.path6End
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }
}
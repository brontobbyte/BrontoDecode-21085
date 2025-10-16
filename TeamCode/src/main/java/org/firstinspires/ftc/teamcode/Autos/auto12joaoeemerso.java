package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

@Config
public class auto12joaoeemerso extends NextFTCOpMode {

    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Shooter.INSTANCE,intake.INSTANCE)

        );
    }

    public static class GeneratedPaths {

        public static Pose Pose1 = new Pose(56.000, 8.000);
        public static Pose Pose2 = new Pose(59.372, 51.175);
        public static Pose Pose3 = new Pose(24.369, 60.258);
        public static Pose Pose4 = new Pose(23.926, 69.563);
        public static Pose Pose5 = new Pose(16.394, 69.563);
        public static Pose Pose6 = new Pose(44.751, 64.246);
        public static Pose Pose7 = new Pose(59.151, 82.855);
        public static Pose Pose8 = new Pose(22.154, 83.298);
        public static Pose Pose9 = new Pose(46.080, 95.705);
        public static Pose Pose10 = new Pose(62.031, 32.788);
        public static Pose Pose11 = new Pose(24.812, 36.997);
        public static Pose Pose12 = new Pose(67.348, 15.729);

        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7;

        public void buildPaths() {

            Path1 = follower.pathBuilder()
                    .addPath(new BezierCurve(Pose1, Pose2, Pose3))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierCurve(Pose3, Pose4, Pose5))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierCurve(Pose5, Pose6, Pose7))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(Pose7, Pose8))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(Pose8, Pose9))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierCurve(Pose9, Pose10, Pose11))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(Pose11, Pose12))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }

    @Override
    public void onInit() { }

    @Override
    public void onWaitForStart() { }

    @Override
    public void onStartButtonPressed() { }
}

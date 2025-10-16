package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous (name = "Auto_PGP")
public class Auto_PGP extends NextFTCOpMode {
    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Shooter.INSTANCE)
        );
    }
    public static class PGPPaths {

        public static double PoseInicialX = 27.468208092485547;
        public static double PoseInicialY = 130.18265895953758;
        public static double ShootInicialPoseX = 43.61618497109827;
        public static double ShootInicialPoseY = 117.8635838150289;
        public static double GoIntakeCurvedPoseX = 41.7849710982659;
        public static double GoIntakeCurvedPoseY = 59.88785046728972;
        public static double GoPoseX = 66.61682242990653;
        public static double GoPoseY = 57.64485981308411;
        public static double IntakePoseX = 23.5;
        public static double IntakePoseY = 59.88785046728972;
        public static double Shoot2PoseX = 49.34579439252336;
        public static double Shoot2PoseY = 94.42990654205607;
        public static double GoIntakePoseX = 47.32710280373831;
        public static double GoIntakePoseY = 72.67289719626169;

        public static double GoIntake2PoseX = 41.7849710982659;
        public static double GoIntake2PoseY = 83.73641618497109;
        public static double Intake2PoseX = 34.5;
        public static double Intake2PoseY = 83.73641618497109;
        public static double GoIntake3PoseX = 52.036416184971;
        public static double GoIntake3PoseY = 63.25186915881503;
        public static double GoIntake3CurvedPoseX = 42.39252336448598;
        public static double GoIntake3CurvedPoseY = 34.990654205607484;
        public static double Intake3PoseX = 30.5;
        public static double Intake3PoseY = 34.990654205607484;
        public static double Shoot3PoseX = 54.50467289719626;
        public static double Shoot3PoseY = 16.822429906542048;


        public static Pose PoseInicial = new Pose(PoseInicialX, PoseInicialY);
        public static Pose ShootInicialPose = new Pose(ShootInicialPoseX, ShootInicialPoseY);
        public static Pose GoIntakeCurvedPose = new Pose(GoIntakeCurvedPoseX, GoIntakeCurvedPoseY);
        public static Pose GoPose = new Pose(GoPoseX, GoPoseY);
        public static Pose IntakePose = new Pose(IntakePoseX, IntakePoseY);
        public static Pose GoIntake2Pose = new Pose(GoIntake2PoseX, GoIntake2PoseY);
        public static Pose Intake2Pose = new Pose(Intake2PoseX, Intake2PoseY);
        public static Pose Shoot2Pose = new Pose(Shoot2PoseX, Shoot2PoseY);
        public static Pose GoIntakePose = new Pose(GoIntakePoseX, GoIntakePoseY);
        public static Pose GoIntake3CurvedPose = new Pose(GoIntake3CurvedPoseX, GoIntake3CurvedPoseY);
        public static Pose GoIntake3Pose = new Pose(GoIntake3PoseX, GoIntake3PoseY);
        public static Pose Intake3Pose = new Pose(Intake3PoseX, Intake3PoseY);
        public static Pose Shoot3Pose = new Pose(Shoot3PoseX, Shoot3PoseY);

        private Path scorePreload;
        private PathChain ShootInicial, GoIntakeCurved, Intake, Shoot2, GoIntake2, Intake2, GoIntake3Curved, Intake3, Shoot3;


        public void buildPathsPGP() {


            ShootInicial = follower.pathBuilder()
                    .addPath(
                            new BezierLine(PoseInicial, ShootInicialPose)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(324), Math.toRadians(324))
                    .build();

            GoIntakeCurved = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    ShootInicialPose,
                                    GoPose,
                                    GoIntakeCurvedPose
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(324), Math.toRadians(180))
                    .build();

            Intake = follower.pathBuilder()
                    .addPath(new BezierLine(GoIntakeCurvedPose, IntakePose ))
                    .setTangentHeadingInterpolation()
                    .build();

            Shoot2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    IntakePose,
                                    GoIntakePose,
                                    Shoot2Pose
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(324))
                    .build();

            GoIntake2 = follower.pathBuilder()
                    .addPath(new BezierLine(Shoot2Pose, GoIntake2Pose))
                    .setLinearHeadingInterpolation(Math.toRadians(324), Math.toRadians(180))
                    .build();

            Intake2 = follower.pathBuilder()
                    .addPath(new BezierLine(GoIntake2Pose, Intake2Pose))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            GoIntake3Curved = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    Intake2Pose,
                                    GoIntake3Pose,
                                    GoIntake3CurvedPose
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Intake3 = follower.pathBuilder()
                    .addPath(new BezierLine(GoIntake3CurvedPose, Intake3Pose))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Shoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(Intake3Pose, Shoot3Pose))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(295))
                    .build();
        }
    }




    @Override public void onInit() { }
    @Override public void onWaitForStart() {

    }
    @Override public void onStartButtonPressed() {

    }
}
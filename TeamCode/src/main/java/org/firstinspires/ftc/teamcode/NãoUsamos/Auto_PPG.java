package org.firstinspires.ftc.teamcode.NãoUsamos;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

@Config
public class Auto_PPG extends NextFTCOpMode {
    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Shooter.INSTANCE)
        );
    }
    public static class PPGPaths {

        public static double PoseInicialX = 27.468208092485547;
        public static double PoseInicialY = 128;
        public static double ShootInicialPoseX = 43.616;
        public static double ShootInicialPoseY = 117.864;
        public static double GoIntakeCurvedPoseX = 41.691;
        public static double GoIntakeCurvedPoseY = 68.233;
        public static double GoIntakeX = 41.785;
        public static double GoIntakeY = 83.736;
        public static double Intake2PoseX = 24.971;
        public static double Intake2PoseY = 83.570;
        public static double Shoot2PoseX = 42.950;
        public static double Shoot2PoseY = 99.884;
        public static double Shoot2CurvedPoseX = 47.840;
        public static double Shoot2CurvedPoseY = 66.693;
        public static double GoPoseX = 49.609;
        public static double GoPoseY = 26.303;
        public static double GoCurvedPoseX = 34.560;
        public static double GoCurvedPoseY = 7.532;
        public static double GoIntake3PoseX = 12.153;
        public static double GoIntake3PoseY = 25.970;
        public static double GoIntake3CurvedPoseX = 26.969;
        public static double GoIntake3CurvedPoseY = 33.295;
        public static double Intake3PoseX = 40.287;
        public static double Intake3PoseY = 25.970;
        public static double Shoot3PoseX = 54.603;
        public static double Shoot3PoseY = 13.817;
        public static double ParkPoseX = 38.622;
        public static double ParkPoseY = 33.295;


        public static Pose PoseInicial = new Pose(PoseInicialX, PoseInicialY);
        public static Pose ShootInicialPose = new Pose(ShootInicialPoseX, ShootInicialPoseY);
        public static Pose GoIntakeCurvedPose = new Pose(GoIntakeCurvedPoseX, GoIntakeCurvedPoseY);
        public static Pose GoIntakePose = new Pose(GoIntakeX, GoIntakeY);
        public static Pose Intake2Pose = new Pose(Intake2PoseX, Intake2PoseY);
        public static Pose Shoot2Pose = new Pose(Shoot2PoseX, Shoot2PoseY);
        public static Pose Shoot2CurvedPose = new Pose(Shoot2CurvedPoseX, Shoot2CurvedPoseY);
        public static Pose GoPose = new Pose(GoPoseX, GoPoseY);
        public static Pose GoCurvedPose = new Pose(GoCurvedPoseX, GoCurvedPoseY);
        public static Pose GoIntake3Pose = new Pose(GoIntake3PoseX, GoIntake3PoseY);
        public static Pose GoIntake3CurvedPose = new Pose(GoIntake3CurvedPoseX, GoIntake3CurvedPoseY);
        public static Pose Intake3Pose = new Pose(Intake3PoseX, Intake3PoseY);
        public static Pose Shoot3Pose = new Pose(Shoot3PoseX, Shoot3PoseY);
        public static Pose ParkPose = new Pose(ParkPoseX, ParkPoseY);
        private Path scorePreload;
        private PathChain ShootInicial, GoIntake2, Intake2, Shoot2, Go, GoIntake3, Intake3, Shoot3, Park;

        public void buildPaths() {

            ShootInicial = follower.pathBuilder()
                    .addPath(
                            new BezierLine(PoseInicial, ShootInicialPose)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-36), Math.toRadians(-36))
                    .build();

            GoIntake2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    ShootInicialPose,
                                    GoIntakeCurvedPose,
                                    GoIntakePose
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-36), Math.toRadians(180))
                    .build();

            Intake2 = follower.pathBuilder()
                    .addPath(new BezierLine(GoIntakePose, Intake2Pose))
                    .setConstantHeadingInterpolation(Math.toRadians(-180))
                    .build();

            Shoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(Intake2Pose, Shoot2Pose))
                    .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-90))
                    .build();

            Go = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    Shoot2Pose,
                                    Shoot2CurvedPose,
                                    GoPose
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-135))
                    .build();

            GoIntake3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    GoPose,
                                    GoCurvedPose,
                                    GoIntake3Pose
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(45))
                    .build();

            Intake3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    GoIntake3Pose,
                                    GoIntake3CurvedPose,
                                    Intake3Pose
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(45))
                    .build();

            Shoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(Intake3Pose, Shoot3Pose))
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                    .build();

            Park = follower.pathBuilder()
                    .addPath(new BezierLine(Shoot3Pose, ParkPose))
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(90))
                    .build();
        }
    }

    @Override public void onInit() { }
    @Override public void onWaitForStart() {

    }
    @Override public void onStartButtonPressed() {

    }
}

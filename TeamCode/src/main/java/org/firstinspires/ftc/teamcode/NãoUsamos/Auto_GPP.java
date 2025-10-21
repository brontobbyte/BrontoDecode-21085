package org.firstinspires.ftc.teamcode.NãoUsamos;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Config
public class Auto_GPP extends NextFTCOpMode {
    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Shooter.INSTANCE)
        );
    }
    public static class GPPPaths {
    public static double PoseInicialX = 27.468208092485547;
    public static double PoseInicialY = 128;
    public static double ShootInicialPoseX = 43.616;
    public static double ShootInicialPoseY = 117.864;
    public static double GoIntakeCurvedPoseX = 73.121;
    public static double GoIntakeCurvedPoseY = 31.402;
    public static double GoIntakeX = 41.495;
    public static double GoIntakeY = 34.766;
    public static double Intake2PoseX = 18.841;
    public static double Intake2PoseY = 34.991;
    public static double Shoot2PoseX = 71.327;
    public static double Shoot2PoseY = 77.383;
    public static double Shoot2CurvedPoseX = 75.364;
    public static double Shoot2CurvedPoseY = 35.888;
    public static double GoPoseX = 35.888;
    public static double GoPoseY = 71.77570093457945;
    public static double GoIntake3PoseX = 35.88785046728972;
    public static double GoIntake3PoseY = 59.214953271028044;
    public static double Intake3PoseX = 18.8411214953271;
    public static double Intake3PoseY = 59.214953271028044;
    public static double Shoot3PoseX = 40.37383177570093;
    public static double Shoot3PoseY = 116.41121495327101;






















    public static Pose PoseInicial = new Pose(PoseInicialX, PoseInicialY);
    public static Pose ShootInicialPose = new Pose(ShootInicialPoseX, ShootInicialPoseY);
    public static Pose GoIntakeCurvedPose = new Pose(GoIntakeCurvedPoseX, GoIntakeCurvedPoseY);
    public static Pose GoIntakePose = new Pose(GoIntakeX, GoIntakeY );
    public static Pose Intake2Pose = new Pose(Intake2PoseX, Intake2PoseY);
    public static Pose Shoot2Pose = new Pose(Shoot2PoseX, Shoot2PoseY);
    public static Pose Shoot2CurvedPose = new Pose(Shoot2CurvedPoseX, Shoot2CurvedPoseY );
    public static Pose GoPose = new Pose(GoPoseX, GoPoseY);
    public static Pose GoIntake3Pose = new Pose(GoIntake3PoseX, GoIntake3PoseY);
    public static Pose Intake3Pose = new Pose(Intake3PoseX, Intake3PoseY);
    public static Pose Shoot3Pose = new Pose(Shoot3PoseX, Shoot3PoseY);

    private Path scorePreload;
    private PathChain ShootInicial, GoIntake2, Intake2, Shoot2, Go, GoIntake3, Intake3, Shoot3, Park;


        public void buildPathsGPP() {


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
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-180))
                    .build();

            GoIntake3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    GoPose,
                                    GoIntake3Pose
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(-180))
                    .build();

            Intake3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    GoIntake3Pose,
                                    Intake3Pose
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(-180))
                    .build();

            Shoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(Intake3Pose, Shoot3Pose))
                    .setConstantHeadingInterpolation(Math.toRadians(-180))
                    .build();
        }
    }
    @Override public void onInit() { }
    @Override public void onWaitForStart() {

    }
    @Override public void onStartButtonPressed() {
    }
}


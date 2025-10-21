package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Config
@Autonomous
public class Auto_12_5 extends NextFTCOpMode {

    //TODO AUTONOMOUS - 12 ARTIFACTS + 5 PATTERN + 3 BASE - 21085 - BRONTOBYTE - BR

    {
        addComponents(
                new PedroComponent(Constants::createFollower)
        );
    }
    private Follower follower;

        // COORDENADAS PARA O PANELS
        public static double PoseInicialX = 62.760693641618495;
        public static double PoseInicialY = 9.489017341040466;
        public static double Intake2CurvedPoseX = 62.734177215189874;
        public static double Intake2CurvedPoseY = 59.930635838150295;
        public static double Intake2PoseX = 21.97456647398844;
        public static double Intake2PoseY = 59.59768786127168;
        public static double OpenGateCurvedPoseX = 22.48101265822785;
        public static double OpenGateCurvedPoseY = 64.92485549132948;
        public static double OpenGatePoseX = 14.126582278481012;
        public static double OpenGatePoseY = 68.75375722543353;
        public static double Shoot2PoseX = 49.67088607594937;
        public static double Shoot2PoseY = 83.73641618497109;
        public static double Shoot2CurvedPoseX = 52.10126582278481;
        public static double Shoot2CurvedPoseY = 59.59768786127168;
        public static double Intake3PoseX = 24.30379746835443;
        public static double Intake3PoseY = 83.73641618497109;
        public static double Shoot3PoseX = 58.09942196531792;
        public static double Shoot3PoseY = 94.7236994219653;
        public static double Shoot3CurvedPoseX = 52.106358381502886;
        public static double Shoot3CurvedPoseY = 87.23236994219653;
        public static double Intake4PoseX = 24.971098265895954;
        public static double Intake4PoseY = 36.4578034682081;
        public static double Intake4CurvedPoseX = 69.91907514450867;
        public static double Intake4CurvedPoseY = 36.291329479768784;
        public static double Shoot4PoseX = 57.11392405063291;
        public static double Shoot4PoseY = 15.482080924855492;







        // POSES COM AS COORDENADAS
        public static Pose PoseInicial = new Pose(PoseInicialX, PoseInicialY);
        public static Pose Intake2CurvedPose = new Pose(Intake2CurvedPoseX, Intake2CurvedPoseY);
        public static Pose Intake2Pose = new Pose(Intake2PoseX, Intake2PoseY);
        public static Pose OpenGateCurvedPose = new Pose(OpenGateCurvedPoseX, OpenGateCurvedPoseY);
        public static Pose OpenGatePose = new Pose(OpenGatePoseX, OpenGatePoseY);
        public static Pose Shoot2Pose = new Pose(Shoot2PoseX, Shoot2PoseY);
        public static Pose Shoot2CurvedPose = new Pose(Shoot2CurvedPoseX, Shoot2CurvedPoseY );
        public static Pose Intake3Pose = new Pose(Intake3PoseX, Intake3PoseY);
        public static Pose Shoot3CurvedPose = new Pose(Shoot3CurvedPoseX, Shoot3CurvedPoseY);
        public static Pose Shoot3Pose = new Pose(Shoot3PoseX, Shoot3PoseY);
        public static Pose Intake4CurvedPose = new Pose(Intake4CurvedPoseX, Intake4CurvedPoseY);

        public static Pose Intake4Pose = new Pose(Intake4PoseX, Intake4PoseY);
        public static Pose Shoot4Pose = new Pose(Shoot4PoseX, Shoot4PoseY);

        private PathChain Intake2, OpenGate, Shoot2, Intake3, Shoot3, Intake4, Shoot4;


        public void buildPathsGPP() {

            //PATHS

            Intake2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    PoseInicial,
                                    Intake2CurvedPose,
                                    Intake2Pose
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            OpenGate = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    Intake2Pose,
                                    OpenGateCurvedPose,
                                    OpenGatePose
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Shoot2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    OpenGatePose,
                                    Shoot2CurvedPose,
                                    Shoot2Pose
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Intake3 = follower.pathBuilder()
                    .addPath(new BezierLine(Shoot2Pose, Intake3Pose))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Shoot3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    Intake3Pose,
                                    Shoot3CurvedPose,
                                    Shoot3Pose
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-90))
                    .build();

            Intake4 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    Shoot3Pose,
                                    Intake4CurvedPose,
                                    Intake4Pose
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Shoot4 = follower.pathBuilder()
                    .addPath(new BezierLine(Intake4Pose, Shoot4Pose))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

    }
    @Override public void onInit() { }
    @Override public void onWaitForStart() {

    }
    @Override public void onStartButtonPressed() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseInicial);
        buildPathsGPP();
        new SequentialGroup(
                new FollowPath(Intake2),
                new FollowPath(OpenGate),
                new FollowPath(Shoot2),
                new FollowPath(Intake3),
                new FollowPath(Shoot3),
                new FollowPath(Intake4),
                new FollowPath(Shoot4)
                );
    }
}


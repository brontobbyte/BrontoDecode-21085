package org.firstinspires.ftc.teamcode.Autos;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.Subsystems.SubsystemsSolversAuto.IntakeSolvers;
import org.firstinspires.ftc.teamcode.Subsystems.SubsystemsSolversAuto.ShooterSolvers;
import org.firstinspires.ftc.teamcode.Subsystems.SubsystemsSolversAuto.TurretSolvers;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous
public class Auto_12_5_Solverslib extends CommandOpMode {

    TelemetryData telemetryData = new TelemetryData(telemetry);
    static TelemetryManager telemetryM;
    static PoseHistory poseHistory;
    private Follower follower;

    //TODO AUTONOMOUS - 12 ARTIFACTS + 5 PATTERN + 3 BASE - 21085 - BRONTOBYTE - BR

    public static int Shoot1PosTurret = 1209;
    public static int Shoot2PosTurret = -700;
    public static int Shoot3PosTurret = -1200;
    public static int Shoot4PosTurret = -1200;

    // COORDENADAS PARA O PANELS
        public static double PoseInicialX = 56;
        public static double PoseInicialY = 11;
        public static double Intake2CurvedPoseX = 62.734177215189874;
        public static double Intake2CurvedPoseY = 52.9306358382;
        public static double Intake2PoseX = 21.97456647398844;
        public static double Intake2PoseY = 61.5976878613;
        public static double OpenGateCurvedPoseX = 22.48101265822785;
        public static double OpenGateCurvedPoseY = 55.9248554913;
        public static double OpenGatePoseX = 18.1265822785;
        public static double OpenGatePoseY = 68.75375722543353;
        public static double Shoot2PoseX = 49.67088607594937;
        public static double Shoot2PoseY = 88.0;
        public static double Shoot2CurvedPoseX = 52.10126582278481;
        public static double Shoot2CurvedPoseY = 59.59768786127168;
        public static double Intake3PoseX = 24.30379746835443;
        public static double Intake3PoseY = 85;
        public static double Shoot3PoseX = 58.09942196531792;
        public static double Shoot3PoseY = 94.7236994219653;
        public static double Shoot3CurvedPoseX = 52.106358381502886;
        public static double Shoot3CurvedPoseY = 87.23236994219653;
        public static double Intake4PoseX = 19;
        public static double Intake4PoseY = 39.0;
        public static double Intake4CurvedPoseX = 69.91907514450867;
        public static double Intake4CurvedPoseY = 36.291329479768784;
        public static double Shoot4PoseX = 57.11392405063291;
        public static double Shoot4PoseY = 25.4820809249;







        // POSES COM AS COORDENADAS
        public static Pose PoseInicial = new Pose(PoseInicialX, PoseInicialY, Math.toRadians(90));
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

        private PathChain Intake2, OpenGate, Shoot2, Intake3, Shoot3, Intake4, Shoot4, teste;


        public void buildPaths() {

            //PATHS

            Intake2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    PoseInicial,
                                    Intake2CurvedPose,
                                    Intake2Pose
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
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

    private InstantCommand intake() {
        return new InstantCommand(() -> {

            new IntakeSolvers(hardwareMap, "motor_intake").Intake();

        });
    }
    private InstantCommand intakeoff() {
        return new InstantCommand(() -> {

            new IntakeSolvers(hardwareMap, "motor_intake").Off();

        });
    }
    private InstantCommand shoot() {
        return new InstantCommand(() -> {

            new ShooterSolvers(hardwareMap, "motor_shooter", "motor_shooter2").On();

        });
    }
    private InstantCommand shootoff() {
        return new InstantCommand(() -> {

            new ShooterSolvers(hardwareMap, "motor_shooter", "motor_shooter2").Off();

        });
    }
    private InstantCommand turretShoot1() {
        return new InstantCommand(() -> {
            new TurretSolvers(hardwareMap, "motor_turret").ShootAuto(Shoot1PosTurret);
        });
    }
    private InstantCommand turretShoot2() {
        return new InstantCommand(() -> {
            new TurretSolvers(hardwareMap, "motor_turret").ShootAuto(Shoot2PosTurret);
        });
    }
    private InstantCommand turretShoot3() {
        return new InstantCommand(() -> {
            new TurretSolvers(hardwareMap, "motor_turret").ShootAuto(Shoot3PosTurret);
        });
    }
    private InstantCommand turretShoot4() {
        return new InstantCommand(() -> {
            new TurretSolvers(hardwareMap, "motor_turret").ShootAuto(Shoot4PosTurret);
        });
    }


    @Override
    public void initialize() {
            super.reset();
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(PoseInicial);
            buildPaths();
            SequentialCommandGroup init = new SequentialCommandGroup(
                    shoot(),
                    turretShoot1()
            );
        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                turretShoot1(),
                //shoot(),
                intake(),
                new WaitCommand(2000),
                new FollowPathCommand(follower, Intake2).setGlobalMaxPower(1),
                intakeoff(),
                new FollowPathCommand(follower, OpenGate),
                turretShoot2(),
                new FollowPathCommand(follower, Shoot2),
                new WaitCommand(1500),
                new FollowPathCommand(follower, Intake3),
                turretShoot3(),
                new FollowPathCommand(follower, Shoot3),
                new WaitCommand(1500),
                new FollowPathCommand(follower, Intake4),
                turretShoot4(),
                new FollowPathCommand(follower, Shoot4)






                );
        follower.update();
        /*if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPathChain(), new Style("", "#3F51B5", 0.0));
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), new Style("", "#3F51B5", 0.0));
        }
        drawPoseHistory(follower.getPoseHistory(), new Style("", "#4CAF50", 0.0));
        drawRobot(follower.getPose(), new Style("", "#4CAF50", 0.0));*/
        schedule(autonomousSequence);
    }

    public void run(){
            super.run();
        follower.update();
        //drawCurrentAndHistory();
        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.update();
    }
}


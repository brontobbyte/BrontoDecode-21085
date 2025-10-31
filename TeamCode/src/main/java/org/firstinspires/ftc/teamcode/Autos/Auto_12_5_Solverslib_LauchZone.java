package org.firstinspires.ftc.teamcode.Autos;


import static org.firstinspires.ftc.teamcode.Subsystems.SubsystemsSolversAuto.ShooterSolvers.vel;

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
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.SubsystemsSolversAuto.IndexerSolvers;
import org.firstinspires.ftc.teamcode.Subsystems.SubsystemsSolversAuto.IntakeSolvers;
import org.firstinspires.ftc.teamcode.Subsystems.SubsystemsSolversAuto.ShooterSolvers;
import org.firstinspires.ftc.teamcode.Subsystems.SubsystemsSolversAuto.TurretSolvers;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous
public class Auto_12_5_Solverslib_LauchZone extends CommandOpMode {
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

        // POSES COM AS COORDENADAS
        public static Pose PoseInicial = new Pose(21.577981651376145, 124.1834862385321, Math.toRadians(145));
        public static Pose Intake2Pose = new Pose(18.495412844036704, 84);
        public static Pose ShootPose = new Pose(59, 84);
        public static Pose Intake3CurvedPose = new Pose(58.128440366972484, 59.0091743119266);
        public static Pose Intake3Pose = new Pose(19.596, 58.789);
        public static Pose Intake4CurvedPose = new Pose(56.587, 36.330);
        public static Pose Intake4Pose = new Pose(21.358, 35.009);
        private PathChain Intake2, Shoot1, Shoot2, Intake3, Shoot3, Intake4, Shoot4, teste;


        public void buildPaths() {

            //PATHS
            Shoot1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    PoseInicial,
                                    ShootPose
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                    .build();
            Intake2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    ShootPose,
                                    Intake2Pose
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Shoot2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    Intake2Pose,
                                    ShootPose
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(230))
                    .build();

            Intake3 = follower.pathBuilder()
                    .addPath(new BezierLine(ShootPose, Intake3Pose))
                    .setTangentHeadingInterpolation()
                    .build();

            Shoot3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    Intake3Pose,
                                    ShootPose
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Intake4 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    ShootPose,
                                    Intake4CurvedPose,
                                    Intake4Pose
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Shoot4 = follower.pathBuilder()
                    .addPath(new BezierLine(Intake4Pose, ShootPose))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

    }

    private InstantCommand intake() {
        return new InstantCommand(() -> {

            new IntakeSolvers(hardwareMap, "motor_intake").Intake();

        });
    }
    private InstantCommand intakeshoot() {
        return new InstantCommand(() -> {

            new IntakeSolvers(hardwareMap, "motor_intake").IntakeShoot();

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
    private InstantCommand turretAutoAlign() {
        return new InstantCommand(() -> {
            new TurretSolvers(hardwareMap, "motor_turret").autoAlign();
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
    private InstantCommand indexer() {
        return new InstantCommand(() -> {
            new IndexerSolvers(hardwareMap, "servo_indexer").On();
        });
    }
    private InstantCommand indexerOff() {
        return new InstantCommand(() -> {
            new IndexerSolvers(hardwareMap, "servo_indexer").Off();
        });
    }


    @Override
    public void initialize() {
            super.reset();
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(PoseInicial);
            buildPaths();
        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new ParallelDeadlineGroup(new FollowPathCommand(follower, Shoot1), /*turretShoot1(),*/ shoot()),
                turretAutoAlign(),
                intake(),
                indexer(),
                new WaitCommand(2000),
                indexerOff(),
                intakeoff(),
                shootoff(),
                new ParallelCommandGroup(intake(),  new FollowPathCommand(follower, Intake2)),
                intakeoff(),
                new ParallelDeadlineGroup(new FollowPathCommand(follower, Shoot2), turretShoot2(), shoot()),
                new ParallelDeadlineGroup(new WaitCommand(4000), intakeshoot()),
                intakeoff(),
                shootoff(),
                new ParallelCommandGroup(intake(),  new FollowPathCommand(follower, Intake3)),
                intakeoff(),
                new ParallelDeadlineGroup(new FollowPathCommand(follower, Shoot3), turretShoot3(), shoot()),
                new ParallelDeadlineGroup(new WaitCommand(4000), intakeshoot()),
                intakeoff(),
                shootoff(),
                new ParallelCommandGroup(intake(),  new FollowPathCommand(follower, Intake4)),
                intakeoff(),
                new ParallelDeadlineGroup(new FollowPathCommand(follower, Shoot4), turretShoot4(), shoot()),
                new ParallelDeadlineGroup(new WaitCommand(4000), intakeshoot()),
                intakeoff(),
                shootoff()
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
        telemetryData.addData("vel", vel);
        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        //telemetryData.addData("VelFlywheel", );
        telemetryData.update();
    }
}


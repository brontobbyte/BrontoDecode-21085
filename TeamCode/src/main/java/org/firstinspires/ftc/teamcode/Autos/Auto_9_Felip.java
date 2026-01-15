package org.firstinspires.ftc.teamcode.Autos;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
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
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.Subsystems.SubsystemsSolversAuto.AnguladorSolvers;
import org.firstinspires.ftc.teamcode.Subsystems.SubsystemsSolversAuto.IndexerSolvers;
import org.firstinspires.ftc.teamcode.Subsystems.SubsystemsSolversAuto.IntakeSolvers;
import org.firstinspires.ftc.teamcode.Subsystems.SubsystemsSolversAuto.ShooterSolvers;
import org.firstinspires.ftc.teamcode.Subsystems.SubsystemsSolversAuto.TurretSolvers;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous
public class Auto_9_Felip extends CommandOpMode {

    TelemetryData telemetryData = new TelemetryData(telemetry);
    static TelemetryManager telemetryM;
    static PoseHistory poseHistory;
    private Follower follower;
    private Motor Turret;

    // AUTONOMO PARA 9 ARTEFATOS - DESAFIO

    double vel;
    public static int Shoot1PosTurret = 100;
    public static int Shoot2PosTurret = 210;
    public static int Shoot3PosTurret = 210;
    public static int Shoot4PosTurret = 210;
    private Motor motorTurret;



    // COORDENADAS PARA O PANELS

    // POSES PARA COORDENADAS

    public static Pose PoseInicial = new Pose (17.723076923076924, 130.7076923076923, Math.toRadians(145));
    public static Pose VaiIntake1 = new Pose (48.33391304347826, 83.14434782608694);
    public static Pose IntakePose1 = new Pose (48.08347826086957, 56.59826086956522);
    public static Pose ShootPose = new Pose (55.38461538461539, 104.12307692307692);
    public static Pose VaiAteLa = new Pose (72.12521739130435, 40.820869565217386);
    public static Pose VaiIntake2 = new Pose (96.66782608695652, 40.820869565217386);
    public static Pose IntakePose2 = new Pose (96.16695652173914, 70.37217391304348);
    private PathChain ShootInicial, GoIntake1, Intake1, Shoot1, VaiLa, GoIntake2, Intake2, Shoot2;

    public void buildPaths () {
        //PATHS

        ShootInicial = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                PoseInicial,
                                ShootPose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(145))
                .build();

        GoIntake1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                ShootPose,
                                VaiIntake1
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(270))
                .build();

        Intake1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                VaiIntake1,
                                IntakePose1
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                .build();

        Shoot1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                IntakePose1,
                                ShootPose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians (145))
                .build();

        VaiLa = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                ShootPose,
                                VaiAteLa
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(0))
                .build();

        GoIntake2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                VaiAteLa,
                                VaiIntake2
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(90))
                .build();

        Intake2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                VaiIntake2,
                                IntakePose2
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                IntakePose2,
                                ShootPose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(145))
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
    private InstantCommand intakevel() {
        return new InstantCommand(() -> {

            new IntakeSolvers(hardwareMap, "motor_intake").vel();

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
            //new TurretSolvers(hardwareMap, "motor_turret").autoAlign();
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
    private InstantCommand indexerReverse() {
        return new InstantCommand(() -> {
            new IndexerSolvers(hardwareMap, "servo_indexer").Reverse();
        });
    }
    private InstantCommand indexerOff() {
        return new InstantCommand(() -> {
            new IndexerSolvers(hardwareMap, "servo_indexer").Off();
        });
    }
    private InstantCommand anguladorAlto() {
        return new InstantCommand(() -> {
            new AnguladorSolvers(hardwareMap, "baba").On();
        });
    }
    private InstantCommand anguladorMedio() {
        return new InstantCommand(() -> {
            new AnguladorSolvers(hardwareMap, "baba").medio();
        });
    }
    private InstantCommand anguladorBaixo() {
        return new InstantCommand(() -> {
            new AnguladorSolvers(hardwareMap, "baba").Off();
        });
    }

    @Override
    public void initialize() {
        motorTurret = new Motor(hardwareMap, "motor_turret");
        //telemetryData.addData("pos", motorTurret.getCurrentPosition());
        telemetry.update();
        super.reset();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseInicial);

        //telemetry.setMsTransmissionInterval(11);

        motorTurret.resetEncoder();
        buildPaths();
        waitForStart();
        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                anguladorMedio(),
                new ParallelCommandGroup(new FollowPathCommand(follower, ShootInicial), shoot()),
                intakeshoot(),
                indexer(),
                new WaitCommand(2000),
                indexerOff(),
                intakeoff(),
                shootoff(),
                anguladorBaixo(),
                new ParallelCommandGroup(new FollowPathCommand(follower, GoIntake1)),
                new ParallelCommandGroup(intake(), new FollowPathCommand(follower, Intake1)),
                indexerOff(),
                intakeoff(),
                new ParallelDeadlineGroup(new FollowPathCommand(follower, Shoot1), shoot()),
                anguladorMedio(),
                intakeshoot(),
                indexer(),
                new WaitCommand(2000),
                indexerOff(),
                anguladorBaixo(),
                shootoff(),
                //indexer(),
                new ParallelCommandGroup(new FollowPathCommand(follower, VaiLa)),
                new ParallelCommandGroup(new FollowPathCommand(follower, GoIntake2)),
                new ParallelCommandGroup(intake(),  new FollowPathCommand(follower, Intake2)),
                indexerOff(),
                intakeoff(),
                new ParallelDeadlineGroup(new FollowPathCommand(follower, Shoot2), shoot()),
                anguladorMedio(),
                intakeshoot(),
                indexer(),
                new WaitCommand(2000),
                anguladorBaixo(),
                indexerOff(),
                intakeoff(),
                shootoff()
        );

        follower.update();
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
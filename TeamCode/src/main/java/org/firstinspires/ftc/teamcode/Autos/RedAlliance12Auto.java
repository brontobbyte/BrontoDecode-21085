package org.firstinspires.ftc.teamcode.Autos;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.RepeatCommand;
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

import java.util.List;

@Configurable
@Autonomous
public class RedAlliance12Auto extends CommandOpMode {
    TelemetryData telemetryData = new TelemetryData(telemetry);
    static TelemetryManager telemetryM;
    static PoseHistory poseHistory;
    private Follower follower;
    private Motor Turret;

    Limelight3A limelight;

    //TODO AUTONOMOUS - 12 ARTIFACTS + 5 PATTERN + 3 BASE - 21085 - BRONTOBYTE - BR
    double vel;
    public static int Shoot1PosTurret = 100;
    public static int Shoot2PosTurret = 210;
    public static int Shoot3PosTurret = 210;
    public static int Shoot4PosTurret = 210;
    private Motor motorTurret;
    // COORDENADAS PARA O PANELS

        // POSES COM AS COORDENADAS
        public static Pose PoseInicial = new Pose(21.577981651376145, 129.1834862385321, Math.toRadians(35));
        public static Pose Intake2Pose = new Pose(16.495412844036704, 80);
        public static Pose ShootPose = new Pose(59, 84);
        public static Pose ShootPoseFinal = new Pose(59, 129);
        public static Pose Intake3CurvedPose = new Pose(68, 54);
        public static Pose Intake3Pose = new Pose(7.596, 55.789);
        public static Pose Intake4CurvedPose = new Pose(70.587, 24);
        public static Pose Intake4Pose = new Pose(7.358, 28);
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
                    .setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))
                    .build();
            Intake2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    ShootPose,
                                    Intake2Pose
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Shoot2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    Intake2Pose,
                                    ShootPose
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(230))
                    .build();

            Intake3 = follower.pathBuilder()
                    .addPath(new BezierCurve(ShootPose,Intake3CurvedPose, Intake3Pose))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
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
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Shoot4 = follower.pathBuilder()
                    .addPath(new BezierLine(Intake4Pose, ShootPoseFinal))
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
    public class autoAlign extends CommandBase {

        // The subsystem the command runs on
        private TurretSolvers turretSolvers;
        boolean end = false;
        @Override
        public void initialize() {
            motorTurret = new Motor(hardwareMap, "motor_turret");
        }

        @Override
        public void execute() {
            LLResult result = limelight.getLatestResult();
            result.getPipelineIndex();
            if (result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    //telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    vel = (fr.getTargetXDegrees()/44);
                }
            }else{
                vel = 0;
                //telemetry.addData("Limelight", "No data available");
            }
            motorTurret.setRunMode(Motor.RunMode.RawPower);
            motorTurret.set(vel);
            if (vel >= -0.1 || vel <= 0.1){
                vel = 0; 
                end = true;
            }
            telemetryData.addData("vel", vel);
            telemetryData.update();
        }


        @Override
        public boolean isFinished() {
            return end;
        }

    }

    @Override
    public void initialize() {
        motorTurret = new Motor(hardwareMap, "motor_turret");
        telemetryData.addData("pos", motorTurret.getCurrentPosition());
        telemetry.update();
        super.reset();
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(PoseInicial);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //telemetry.setMsTransmissionInterval(11);
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(1); // Switch to pipeline number 0
        limelight.start();
        motorTurret.resetEncoder();
        buildPaths();
        waitForStart();
        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                turretShoot1(),
                anguladorMedio(),
                new ParallelDeadlineGroup(new FollowPathCommand(follower, Shoot1), shoot()),
                new RepeatCommand(new autoAlign(), 50),
                intakeshoot(),
                indexer(),
                new WaitCommand(2000),
                indexerOff(),
                intakeoff(),
                shootoff(),
                anguladorBaixo(),
                new ParallelCommandGroup(intake(),new FollowPathCommand(follower, Intake2)),
                indexerOff(),
                intakeoff(),
                new ParallelDeadlineGroup(new FollowPathCommand(follower, Shoot2), shoot(), turretShoot2()),
                anguladorMedio(),
                new RepeatCommand(new autoAlign(), 50),
                intakeshoot(),
                indexer(),
                new WaitCommand(2000),
                indexerOff(),
                anguladorBaixo(),
                shootoff(),
                //indexer(),
                new ParallelCommandGroup(intake(),  new FollowPathCommand(follower, Intake3)),
                intakeoff(),
                shoot(),
                anguladorMedio(),
                new ParallelDeadlineGroup(new FollowPathCommand(follower, Shoot3), turretShoot3()),
                new RepeatCommand(new autoAlign(), 50),
                intakeshoot(),
                indexer(),
                new WaitCommand(2000),
                anguladorBaixo(),
                indexerOff(),
                intakeoff(),
                shootoff(),
                new ParallelCommandGroup(intake(),  new FollowPathCommand(follower, Intake4)),
                intakeoff(),
                anguladorBaixo(),
                anguladorMedio(),
                new ParallelDeadlineGroup(new FollowPathCommand(follower, Shoot4), turretShoot4(), shoot()),
                new RepeatCommand(new autoAlign(), 50),
                intakeshoot(),
                indexer(),
                new WaitCommand(2000),
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


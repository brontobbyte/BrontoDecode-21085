package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.Calculos.angleToEncoderTicks;
import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.Calculos.encoderTicksToAngle;
import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.poseInicial;
import static org.firstinspires.ftc.teamcode.Subsystems.Shooter.Fka;
import static org.firstinspires.ftc.teamcode.Subsystems.Shooter.Fkd;
import static org.firstinspires.ftc.teamcode.Subsystems.Shooter.Fki;
import static org.firstinspires.ftc.teamcode.Subsystems.Shooter.Fkp;
import static org.firstinspires.ftc.teamcode.Subsystems.Shooter.Fks;
import static org.firstinspires.ftc.teamcode.Subsystems.Shooter.Fkv;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.contador;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.toTurn;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.turretAngle;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import com.bylazar.telemetry.PanelsTelemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.localization.localizers.ThreeWheelIMULocalizer;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.paths.callbacks.ParametricCallback;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Constants.AutoCommands;
import org.firstinspires.ftc.teamcode.Constants.AutoPaths;
import org.firstinspires.ftc.teamcode.Constants.AutoPoses;
import org.firstinspires.ftc.teamcode.Constants.PoseManager;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lock;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.time.Duration;
import java.util.List;

//TODO AUTONOMOUS - 21 ARTIFACTS CLASSIFIED + 3 BASE - 21085 - BRONTOBYTE - BR
@Configurable
@Autonomous
public class Auto21 extends NextFTCOpMode {
    {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Lock.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }
    MotorGroup Flywheel = new MotorGroup(
            new MotorEx("f1"),
            new MotorEx("f2")
    );
    //PanelsTelemetry panelsTelemetry;
    public static double Fkp1 = 0.0003;
    public static double Fki1 = 0.0001;
    public static double Fkd1 = 0.001;
    public static double Fks1 = 0.0003;
    public static double Fka1 = 0.0002;
    public static double Fkv1 = 0.000607;
    private Follower follower;
    private Localizer localizer;
    public static double vel = 980;

    Limelight3A limelight;
    private double angleLL = 0;
    private PathChain InicialIntake, preintake, Shoot1, Shoot2, Shoot3, AbrirGate, Gate, ShootDoGate, Intake2, Intake3, preintake2, Intake4, Shoot4, Intake5;

    public void buildPaths() {
        InicialIntake = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                poseInicial,
                                //AutoPoses.intakeCurvedPose,
                                new Pose(39, 103)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                .build();
        Shoot1 = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.intakePose,
                                AutoPoses.shootPose1
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        AbrirGate = PedroComponent.follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(29.51,  65),
                                new Pose(2, 65),
                                new Pose(29, 65)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Gate = PedroComponent.follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(17.641, 54),
                                new Pose(40, 54)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        ShootDoGate = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(40, 54),
                                new Pose(16, 63)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                .build();
        preintake = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                AutoPoses.intakePose,
                                new Pose(60, 70)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        Intake2 = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                AutoPoses.shootPose2,
                                AutoPoses.intake2Pose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        Shoot2 = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.intake2Pose,
                                AutoPoses.intakePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        Intake3 = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(42, 60),
                                AutoPoses.intake3Pose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        Shoot3 = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.intake3Pose,
                                AutoPoses.intakePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        Intake4 = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(49, 35),
                                new Pose(9, 35)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        Shoot4 = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(15, 35),
                                AutoPoses.intakePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        preintake2 = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.intakePose,
                                new Pose(49, 35)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        Intake5 = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(30, 45),
                                new Pose(10, 45),
                                new Pose(10, 0)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                .build();


    }

    @Override
    public void onInit() {
        Intake.INSTANCE.initialize();
        Intake.INSTANCE.intake.invoke();
        Intake.INSTANCE.stop.invoke();
        Lock.INSTANCE.closed.invoke();
        Lock.INSTANCE.open.invoke();
        //Turret.INSTANCE.reset();
        angleLL = 0;
        PedroComponent.follower().setStartingPose(poseInicial);
        Pose poseAtual = PedroComponent.follower().getPose();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(400);
        limelight.pipelineSwitch(4);
        limelight.start();
        CommandManager.INSTANCE.scheduleCommand(Lock.INSTANCE.open);

    }

    @Override
    public void onWaitForStart() {
        Pose poseAtual = PedroComponent.follower().getPose();
        LLStatus status = limelight.getStatus();
        LLResult result = limelight.getLatestResult();
        result.getPipelineIndex();
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                this.angleLL = (-fr.getTargetXDegrees());

            }
        } else {
            this.angleLL = 0.0;
            telemetry.addData("Limelight", "No data available");
        }
        //Turret.INSTANCE.setPoseTracker(poseAtual.getX(), poseAtual.getY(), Math.toDegrees(poseAtual.getHeading()), angleLL);
        //Turret.INSTANCE.periodic();
        telemetry.addData("contador", contador);
        telemetry.addData("angleLL", angleLL);
        telemetry.addData("velo", Flywheel.getVelocity());
        //panelsTelemetry.getTelemetry().addData("velo", Flywheel.getVelocity());
        //panelsTelemetry.getTelemetry().update();
        telemetry.addData("heading", poseAtual.getHeading());
        telemetry.addData("x", poseAtual.getX());
        telemetry.update();
        ControlSystem controlSystem = ControlSystem.builder()
                .velPid(Fkp1, Fki1, Fkd1)
                .basicFF(Fkv1, Fka1, Fks1)
                .build();

        controlSystem.setGoal(new KineticState(0, vel));

        double power = controlSystem.calculate(new KineticState(
                Flywheel.getCurrentPosition(),
                Flywheel.getVelocity()));

        Flywheel.setPower(power);

    }

    @Override
    public void onStartButtonPressed() {
        buildPaths();
        PedroComponent.follower().setStartingPose(poseInicial);
        CommandManager.INSTANCE.scheduleCommand(
                new SequentialGroup(
                        new ParallelGroup(
                                Lock.INSTANCE.open,
                                new FollowPath(InicialIntake)

                        ),
                        Intake.INSTANCE.intake,
                        new Delay(0.8),
                        Lock.INSTANCE.closed,
                        new FollowPath(Intake2),
                        Intake.INSTANCE.stop,
                        new FollowPath(Shoot2),
                        Intake.INSTANCE.intake,
                        new Delay(0.1),
                        Lock.INSTANCE.open,
                        new Delay(1),
                        Lock.INSTANCE.closed,
                        new FollowPath(Intake3),
                        new FollowPath(Gate),
                        new FollowPath(ShootDoGate),
                        Intake.INSTANCE.stop,
                        new FollowPath(Shoot3),
                        Intake.INSTANCE.intake,
                        Lock.INSTANCE.open,
                        new Delay(1),
                        Lock.INSTANCE.closed,
                        //new FollowPath(AbrirGate, true, 0.9).setInterruptible(true),
                        //new FollowPath(Gate),
                        //new FollowPath(Shoot3),
                        //Lock.INSTANCE.open,
                        //new Delay(1),
                        //Lock.INSTANCE.closed,
                        new FollowPath(preintake2),
                        new FollowPath(Intake4),
                        Intake.INSTANCE.stop,
                        new FollowPath(Shoot4),
                        Intake.INSTANCE.intake,
                        Lock.INSTANCE.open,
                        new Delay(1),
                        Lock.INSTANCE.closed,
                        new FollowPath(Intake5),
                        Intake.INSTANCE.stop,
                        new FollowPath(Shoot3),
                        Intake.INSTANCE.intake,
                        Lock.INSTANCE.open
                        /*new FollowPath(AbrirGate, true, 0.75).setInterruptible(true),
                        new FollowPath(Gate, true, 1.0),
                        new FollowPath(Shoot3),
                        Lock.INSTANCE.open,
                        new Delay(1),
                        Lock.INSTANCE.closed,
                        new FollowPath(AbrirGate, true, 0.75).setInterruptible(true),
                        new FollowPath(Gate, true, 1.0),

                        //new Delay(1000),
                        new ParallelGroup(
                                new FollowPath(ShootDoGate),
                                Lock.INSTANCE.open
                        ),
                        Intake.INSTANCE.shooting,
                        new FollowPath(AbrirGate, true, 0.9).setInterruptible(true),
                        new FollowPath(Gate, true, 1.0).setInterruptible(true),
                        Intake.INSTANCE.stop.afterTime(1),
                        new ParallelGroup(
                                new FollowPath(ShootDoGate),
                                Lock.INSTANCE.open
                        ),
                        Intake.INSTANCE.shooting,
                        new FollowPath(Intake2),
                        new ParallelGroup(
                                new FollowPath(Shoot2),
                                Lock.INSTANCE.open
                        ),
                        Intake.INSTANCE.shooting,
                        new FollowPath(Intake3),
                        new ParallelGroup(
                                new FollowPath(Shoot3),
                                Lock.INSTANCE.open
                        ),
                        Intake.INSTANCE.shooting*/
                )
        );
        PedroComponent.follower().update();
    }

    @Override
    public void onUpdate() {
        ControlSystem controlSystem = ControlSystem.builder()
                .velPid(Fkp1, Fki1, Fkd1)
                .basicFF(Fkv1, Fka1, Fks1)
                .build();

        controlSystem.setGoal(new KineticState(0, vel));

        double power = controlSystem.calculate(new KineticState(
                Flywheel.getCurrentPosition(),
                Flywheel.getVelocity()));

        Flywheel.setPower(power);
        Pose poseAtual = PedroComponent.follower().poseTracker.getPose();
        LLStatus status = limelight.getStatus();
        LLResult result = limelight.getLatestResult();
        result.getPipelineIndex();
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                angleLL = (-fr.getTargetXDegrees());
            }
        } else {
            this.angleLL = 0.0;
            telemetry.addData("Limelight", "No data available");
        }
        //Turret.INSTANCE.setPoseTracker(poseAtual.getX(), poseAtual.getY(), Math.toDegrees(poseAtual.getHeading()), angleLL);
        //Turret.INSTANCE.periodic();
        telemetry.addData("velo", Flywheel.getVelocity());
        telemetry.addData("heading", poseAtual.getHeading());
        telemetry.addData("turretAngle", turretAngle);
        telemetry.addData("contador", contador);
        telemetry.addData("destinationAngle", Turret.destinationAngle);
        telemetry.addData("y", poseAtual.getY());
        telemetry.addData("x", poseAtual.getX());
        telemetry.update();
    }

    @Override
    public void onStop() {
        PoseManager.currentPose = PedroComponent.follower().getPose();
    }
}
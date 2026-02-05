package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.Calculos.angleToEncoderTicks;
import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.Calculos.encoderTicksToAngle;
import static org.firstinspires.ftc.teamcode.Constants.AutoPaths.InicialIntake;
import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.PoseInicialfar;

import static org.firstinspires.ftc.teamcode.Subsystems.Turret.contador;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.turretAngle;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
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

@Configurable
@Autonomous
public class farazul extends NextFTCOpMode {
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
    public static double Fkp1 = 0.003;
    public static double Fki1 = 0.0001;
    public static double Fkd1 = 0.001;
    public static double Fks1 = 0.0003;
    public static double Fka1 = 0.0002;
    public static double Fkv1 = 0.000807;
    private Follower follower;
    private Localizer localizer;
    public static double vel = 1230;

    Limelight3A limelight;
    private double angleLL = 0;
    private Paths paths = new Paths();

    public static class Paths {
        public PathChain humanp;
        public PathChain shoothumanp;
        public PathChain fileira3;
        public PathChain shoot3fileira;
        public PathChain gate1;
        public PathChain shootgate;
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
        PedroComponent.follower().setStartingPose(PoseInicialfar);
        Pose poseAtual = PedroComponent.follower().getPose();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(400);
        limelight.pipelineSwitch(4);
        limelight.start();
        CommandManager.INSTANCE.scheduleCommand(Lock.INSTANCE.open);
        Follower follower = PedroComponent.follower();
        paths.humanp = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 8.000),
                                new Pose(4.541, 5.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        paths.shoothumanp = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9.541, 8.037),
                                PoseInicialfar
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        paths.fileira3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(62.661, 24.587),
                                new Pose(49.541, 34.569),
                                new Pose(25.642, 34.936)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        paths.shoot3fileira = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(25.642, 34.936),
                                PoseInicialfar                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        paths.gate1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(61.826, 24.881),
                                new Pose(9.326, 20.151),
                                new Pose(8.716, 40.927)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        paths.shootgate = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(8.716, 40.927),
                                PoseInicialfar
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
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
        PedroComponent.follower().setStartingPose(PoseInicialfar);
        CommandManager.INSTANCE.scheduleCommand(
                new SequentialGroup(
                        Intake.INSTANCE.shooting,
                        new Delay(1),
                        Lock.INSTANCE.closed,
                        Lock.INSTANCE.closed,
                        Lock.INSTANCE.closed,
                        new Delay(1),
                        new FollowPath(paths.humanp, true, 0.5),
                        Intake.INSTANCE.stop,
                        new FollowPath(paths.shoothumanp),
                        Intake.INSTANCE.shooting,
                        Lock.INSTANCE.open,
                        new Delay(1),
                        Lock.INSTANCE.closed,
                        new FollowPath(paths.fileira3),
                        Intake.INSTANCE.stop,
                        new FollowPath(paths.shoot3fileira),
                        Lock.INSTANCE.open,
                        Intake.INSTANCE.intake,
                        new Delay(1),
                        Lock.INSTANCE.closed,
                        new FollowPath(paths.gate1),
                        Intake.INSTANCE.intake,
                        new Delay(3),
                        new FollowPath(paths.shootgate),
                        Intake.INSTANCE.intake,
                        Lock.INSTANCE.open,
                        new Delay(5)




                       /* new FollowPath(Shoot2),
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
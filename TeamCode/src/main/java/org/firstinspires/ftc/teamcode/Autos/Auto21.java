package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.Calculos.angleToEncoderTicks;
import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.Calculos.encoderTicksToAngle;
import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.poseInicial;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.toTurn;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.turretAngle;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

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
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Constants.AutoCommands;
import org.firstinspires.ftc.teamcode.Constants.AutoPaths;
import org.firstinspires.ftc.teamcode.Constants.AutoPoses;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lock;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

//TODO AUTONOMOUS - 21 ARTIFACTS CLASSIFIED + 3 BASE - 21085 - BRONTOBYTE - BR
@Autonomous
public class Auto21 extends NextFTCOpMode{
    {
        addComponents(
                new SubsystemComponent(Turret.INSTANCE, Shooter.INSTANCE, Intake.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }
    private Follower follower;
    private Localizer localizer;
    Limelight3A limelight;
    private double angleLL = 0;
    private PathChain InicialIntake, Shoot1, Shoot2, Shoot3, Gate, ShootDoGate, Intake2, Intake3;
    public void buildPaths() {
        InicialIntake = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                poseInicial,
                                AutoPoses.intakeCurvedPose,
                                AutoPoses.intakePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                .build();
        Shoot1        = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.intakePose,
                                AutoPoses.shootPose1
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        Gate          =  PedroComponent.follower().pathBuilder()

                .addPath(
                        new BezierLine(
                                AutoPoses.shootPose1,
                                AutoPoses.intakeGatePose
                        )
                )
                .setConstraints(new PathConstraints(3, 100, 1, 1))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(148))
                .build();
        ShootDoGate   =  PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.intakeGatePose,
                                AutoPoses.shootPose2
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                .build();
        Intake2       = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                AutoPoses.shootPose2,
                                AutoPoses.intake2CurvedPose,
                                AutoPoses.intake2Pose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        Shoot2        =   PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.intake2Pose,
                                AutoPoses.shootPose1
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        Intake3       =  PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                AutoPoses.shootPose2,
                                AutoPoses.intake3CurvedPose,
                                AutoPoses.intake3Pose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        Shoot3        =   PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierLine(
                                AutoPoses.intake3Pose,
                                AutoPoses.shootPose1
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

    }
    @Override public void onInit() {
        angleLL = 0;
        PedroComponent.follower().setStartingPose(poseInicial);
        Pose poseAtual = PedroComponent.follower().getPose();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(400);
        limelight.pipelineSwitch(4);
        limelight.start();
    }
    @Override public void onWaitForStart() {
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
        }else{
            telemetry.addData("Limelight", "No data available");
        }
        Turret.INSTANCE.setPoseTracker(poseAtual.getX(), poseAtual.getY(), Math.toDegrees(poseAtual.getHeading()), angleLL);
        Turret.INSTANCE.periodic();
        telemetry.addData("heading", poseAtual.getHeading());
        telemetry.addData("x", poseAtual.getX());
        telemetry.update();

    }
    @Override public void onStartButtonPressed() {
        buildPaths();
        PedroComponent.follower().setStartingPose(poseInicial);
        CommandManager.INSTANCE.scheduleCommand(
                new ParallelGroup(
                        new SequentialGroup(
                                new ParallelGroup(
                                        new FollowPath(InicialIntake),
                                        Lock.INSTANCE.open,
                                        Intake.INSTANCE.shooting.afterTime(1)
                                ),
                                Lock.INSTANCE.closed.afterTime(0.5),
                                new ParallelGroup(
                                        new FollowPath(Shoot1),
                                        Lock.INSTANCE.open
                                ),
                                Intake.INSTANCE.shooting,
                                Lock.INSTANCE.closed.afterTime(0.5),
                                new FollowPath(Gate, true, 0.75).setInterruptible(true),
                                Intake.INSTANCE.stop.afterTime(1),
                                new ParallelGroup(
                                        new FollowPath(ShootDoGate),
                                        Lock.INSTANCE.open
                                ),
                                Intake.INSTANCE.shooting,
                                new FollowPath(Gate, true, 0.75).setInterruptible(true),
                                new ParallelGroup(
                                        new FollowPath(ShootDoGate),
                                        Lock.INSTANCE.open
                                ),
                                Intake.INSTANCE.shooting,
                                Lock.INSTANCE.closed.afterTime(0.5),
                                new FollowPath(Gate, true, 0.75).setInterruptible(true),
                                Intake.INSTANCE.stop.afterTime(1),
                                new ParallelGroup(
                                        new FollowPath(ShootDoGate),
                                        Lock.INSTANCE.open
                                ),
                                Intake.INSTANCE.shooting,
                                Lock.INSTANCE.closed.afterTime(0.5),
                                new FollowPath(Intake2),
                                new ParallelGroup(
                                        new FollowPath(Shoot2),
                                        Lock.INSTANCE.open
                                ),
                                Intake.INSTANCE.shooting,
                                Lock.INSTANCE.closed.afterTime(0.5),
                                new FollowPath(Intake3),
                                new ParallelGroup(
                                        new FollowPath(Shoot3),
                                        Lock.INSTANCE.open
                                ),
                                Intake.INSTANCE.shooting
                        )
                )
        );
        PedroComponent.follower().update();
    }
    @Override public void onUpdate() {
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
        }else{
            telemetry.addData("Limelight", "No data available");
        }
        Turret.INSTANCE.setPoseTracker(poseAtual.getX(), poseAtual.getY(), Math.toDegrees(poseAtual.getHeading()), angleLL);
        Turret.INSTANCE.periodic();
        telemetry.addData("heading", poseAtual.getHeading());
        telemetry.addData("turretAngle", turretAngle);
        telemetry.addData("destinationAngle", Turret.destinationAngle);
        telemetry.addData("y", poseAtual.getY());
        telemetry.addData("x", poseAtual.getX());
        telemetry.update();
    }
    @Override public void onStop() { }
}


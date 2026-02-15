package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.goalPose;
import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.poseInicial;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.toTurn;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.turretAngle;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Constants.AutoPathsAzul;
import org.firstinspires.ftc.teamcode.Constants.PoseManager;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lock;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Constants.LimelightHelper;
import org.firstinspires.ftc.teamcode.Constants.TelemetryHelper;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// TODO AUTONOMOUS - 21 ARTIFACTS CLASSIFIED AZUL + 3 BASE - 21085 - BRONTOBYTE - BR
@Configurable
@Autonomous
public class Auto21Azul extends NextFTCOpMode {
    {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Lock.INSTANCE),
                new SubsystemComponent(Shooter.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    public static boolean debugMode = true;

    Limelight3A limelight;
    private double angleLL = 0;

    @Override
    public void onInit() {
        Intake.INSTANCE.initialize();
        Intake.INSTANCE.intake.invoke();
        Intake.INSTANCE.stop.invoke();
        Lock.INSTANCE.closed.invoke();
        Lock.INSTANCE.open.invoke();
        angleLL = 0;
        PedroComponent.follower().setStartingPose(poseInicial);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(400);
        limelight.pipelineSwitch(4);
        limelight.start();
        CommandManager.INSTANCE.scheduleCommand(Lock.INSTANCE.open);
    }

    @Override
    public void onWaitForStart() {
        angleLL = LimelightHelper.updateAngleLL(limelight);

        double distanceToGoal = PedroComponent.follower().getPose().distanceFrom(goalPose);
        Shooter.INSTANCE.setGoalDistance(distanceToGoal);

        TelemetryHelper.addCommonTelemetry(telemetry, PedroComponent.follower().getPose(),
                Shooter.INSTANCE.getVelocity(), turretAngle, Turret.destinationAngle, toTurn,
                limelight, angleLL, debugMode, distanceToGoal, 0.0, 0.0, 0.0);
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        PedroComponent.follower().setStartingPose(poseInicial);
        CommandManager.INSTANCE.scheduleCommand(
                new SequentialGroup(
                        new FollowPath(AutoPathsAzul.InicialIntake),
                        new FollowPath(AutoPathsAzul.Intake2),
                        new FollowPath(AutoPathsAzul.Shoot3)
                )
        );
        PedroComponent.follower().update();
    }

    @Override
    public void onUpdate() {
        angleLL = LimelightHelper.updateAngleLL(limelight);

        double distanceToGoal = PedroComponent.follower().getPose().distanceFrom(goalPose);
        Shooter.INSTANCE.setGoalDistance(distanceToGoal);

        Shooter.INSTANCE.periodic();

        TelemetryHelper.addCommonTelemetry(telemetry, PedroComponent.follower().getPose(),
                Shooter.INSTANCE.getVelocity(), turretAngle, Turret.destinationAngle, toTurn,
                limelight, angleLL, debugMode, distanceToGoal, 0.0, 0.0, 0.0);
        telemetry.update();
    }

    @Override
    public void onStop() {
        PoseManager.currentPose = PedroComponent.follower().getPose();
    }

    private SequentialGroup shoot() {
        return new SequentialGroup(
                Lock.INSTANCE.open,
                Intake.INSTANCE.shooting,
                new Delay(1),
                Lock.INSTANCE.closed.and(Intake.INSTANCE.stop)
        );
    }

    private SequentialGroup intake() {
        return new SequentialGroup(
                Lock.INSTANCE.closed,
                Intake.INSTANCE.intake
        );
    }
}
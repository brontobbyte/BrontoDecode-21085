package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.goalPose;
import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.poseInicial;
import static org.firstinspires.ftc.teamcode.Constants.PoseManager.currentPose;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;

import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lock;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Constants.LimelightHelper;
import org.firstinspires.ftc.teamcode.Constants.TelemetryHelper;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Objects;

@Configurable
@TeleOp(name = "TeleOpAzul")
public class TeleOpAzul extends NextFTCOpMode {
    public TeleOpAzul() {
        addComponents(
                new SubsystemComponent(Turret.INSTANCE),
                new SubsystemComponent(Intake.INSTANCE),
                new SubsystemComponent(Lock.INSTANCE),
                new SubsystemComponent(Shooter.INSTANCE),
                new SubsystemComponent(Hood.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private Limelight3A limelight;
    private double angleLL = 0;
    private DriverControlledCommand driverControlled;
    public static double goalx = 15;
    public static double goaly = 160;
    public static boolean debugMode = true;
    public static boolean align = false;
    public static double K = 0.15; // quanto confiar na MegaTag (0.05–0.2 é bom)
    double megaX;
    double megaY;
    double currentX;
    double currentY;

    FTCCoordinates ftccoords;

    @Override
    public void onInit() {
        Lock.INSTANCE.closed.invoke();
        angleLL = 0;
        PedroComponent.follower().setStartingPose(poseInicial);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(400);
        limelight.pipelineSwitch(4);
        limelight.start();
        PedroComponent.follower().update();

    }

    @Override
    public void onStartButtonPressed() {
        PedroComponent.follower().setStartingPose(poseInicial);
        PedroComponent.follower().update();
        driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX().negate(),
                false
        );
        driverControlled.schedule();

        Gamepads.gamepad1().leftBumper().whenBecomesTrue(() -> {
            //Lock.INSTANCE.closed.schedule();
        });
        Gamepads.gamepad1().leftBumper().whenTrue(() -> {
            Intake.INSTANCE.intake.schedule();
        });
        Gamepads.gamepad1().leftBumper().whenBecomesFalse(() -> {
            Intake.INSTANCE.stop.schedule();
        });
        Gamepads.gamepad1().b().whenTrue(() -> {
            Intake.INSTANCE.reversed.schedule();
        });
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> {
            new SequentialGroup(
                    Lock.INSTANCE.open,
                    Intake.INSTANCE.intake,
                    new Delay(1),
                    Intake.INSTANCE.stop
            ).schedule();
        });
    }

    @Override
    public void onUpdate() {
        PedroComponent.follower().update();
        Pose poseAtual = PedroComponent.follower().poseTracker.getPose();
        driverControlled.update();
        angleLL = LimelightHelper.updateAngleLL(limelight);
        LLResult result = limelight.getLatestResult();
        telemetry.update();
        Turret.INSTANCE.setPoseTracker(poseAtual.getX(), poseAtual.getY(), Math.toDegrees(poseAtual.getHeading()), angleLL, true, telemetry);
        Turret.INSTANCE.periodic();

        double distanceToGoal = PedroComponent.follower().poseTracker.getPose().distanceFrom(goalPose);
        Shooter.INSTANCE.setGoalDistance(distanceToGoal);
        Hood.INSTANCE.periodic();

        TelemetryHelper.addCommonTelemetry(telemetry, poseAtual, Shooter.INSTANCE.getVelocity(),
                Turret.turretAngle, Turret.destinationAngle, Turret.INSTANCE.toTurn,
                limelight, angleLL, debugMode, distanceToGoal, 0, 0, 0);
        telemetry.update();
    }
}
package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.goalPose;
import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.poseInicial;
import static org.firstinspires.ftc.teamcode.Constants.PoseManager.currentPose;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;

import org.firstinspires.ftc.teamcode.Constants.PoseManager;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lock;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Constants.PathsTeleop;

import java.util.List;

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
    private boolean intakeRunning = false;
    private PathsTeleop paths;
    private boolean followingPath1 = false;
    private boolean followingPath2 = false;
    private DriverControlledCommand driverControlled;

    public static double goalx = 10;
    public static double goaly = 137;

    @Override
    public void onInit() {
        angleLL = 0;
        PedroComponent.follower().setStartingPose(poseInicial);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(400);
        limelight.pipelineSwitch(4);
        limelight.start();

        paths = new PathsTeleop(PedroComponent.follower());
    }

    @Override
    public void onStartButtonPressed() {
        driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX(),
                false
        );
        driverControlled.schedule();

        Gamepads.gamepad1().leftBumper().whenBecomesTrue(() -> {
            intakeRunning = !intakeRunning;
            if (intakeRunning) {
                Intake.INSTANCE.intake.schedule();
                Lock.INSTANCE.closed.schedule();
            } else {
                Intake.INSTANCE.stop.schedule();
            }
        });

        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> {
            intakeRunning = false;
            Intake.INSTANCE.intake.schedule();
            Lock.INSTANCE.open.schedule();
        });

        Gamepads.gamepad1().b().whenBecomesTrue(() -> {
            if (driverControlled != null) {
                driverControlled.cancel();
            }
            followingPath1 = true;
            PedroComponent.follower().followPath(paths.Gate);
        });
    }

    @Override
    public void onUpdate() {
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
            telemetry.addData("Limelight", "No data available");
        }

        Turret.INSTANCE.setPoseTracker(poseAtual.getX(), poseAtual.getY(), Math.toDegrees(poseAtual.getHeading()), angleLL);
        Turret.INSTANCE.periodic();

        double distanceToGoal = PedroComponent.follower().poseTracker.getPose().distanceFrom(goalPose);
        Shooter.INSTANCE.setGoalDistance(distanceToGoal);
        Hood.INSTANCE.setGoalDistance(distanceToGoal);

        Shooter.INSTANCE.periodic();
        Hood.INSTANCE.periodic();

        telemetry.addData("heading", poseAtual.getHeading());
        telemetry.addData("turretAngle", Turret.turretAngle);
        telemetry.addData("destinationAngle", Turret.destinationAngle);
        telemetry.addData("y", poseAtual.getY());
        telemetry.addData("x", poseAtual.getX());
        telemetry.addData("intakeRunning", intakeRunning);
        telemetry.addData("distanceToGoal", distanceToGoal);
        telemetry.addData("flywheelVelocity", Shooter.INSTANCE.getVelocity());
        telemetry.addData("Shooter Goal Distance", distanceToGoal);


        double leftStickY = Gamepads.gamepad1().leftStickY().get();
        double leftStickX = Gamepads.gamepad1().leftStickX().get();
        double rightStickX = Gamepads.gamepad1().rightStickX().get();
        boolean joystickMoved = Math.abs(leftStickY) > 0.1 || Math.abs(leftStickX) > 0.1 || Math.abs(rightStickX) > 0.1;

        if ((followingPath1 || followingPath2) && joystickMoved) {
            PedroComponent.follower().breakFollowing();
            followingPath1 = false;
            followingPath2 = false;
            driverControlled = new PedroDriverControlled(
                    Gamepads.gamepad1().leftStickY(),
                    Gamepads.gamepad1().leftStickX(),
                    Gamepads.gamepad1().rightStickX(),
                    false
            );
            driverControlled.schedule();
        }

        if (followingPath1 && !PedroComponent.follower().isBusy()) {
            followingPath1 = false;
            followingPath2 = true;
            PedroComponent.follower().followPath(paths.Intake);
        }

        telemetry.update();
    }
}
package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.goalPose;
import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.poseInicial;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.PoseTracker;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lock;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.impl.MotorEx;

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
    public Localizer localizer;
    private double angleLL = 0;
    private double offset = 0;
    private DriverControlledCommand driverControlled;
    private static final MotorEx motor = new MotorEx("turret");

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
    }

    @Override
    public void onStartButtonPressed() {
        Command driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX().negate(),
                false

        );
        driverControlled.schedule();
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(() -> {
            Lock.INSTANCE.closed.schedule();
        });
        Gamepads.gamepad1().leftBumper().whenTrue(() -> {
            Intake.INSTANCE.intake.schedule();
        });
        Gamepads.gamepad1().leftBumper().whenBecomesFalse(() -> {
            Intake.INSTANCE.stop.schedule();
        });

        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> {
            new SequentialGroup(
                    Lock.INSTANCE.open,
                    new Delay(0.5),
                    Intake.INSTANCE.intake,
                    new Delay(1),
                    Intake.INSTANCE.stop
            ).schedule();
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
            angleLL = 0;

            telemetry.addData("Limelight", "No data available");
        }


        Hood.INSTANCE.setGoalDistance(poseAtual.distanceFrom(goalPose));

        Turret.INSTANCE.setPoseTracker(poseAtual.getX(), poseAtual.getY(), Math.toDegrees(poseAtual.getHeading()), angleLL, offset);
        Turret.INSTANCE.periodic();
        if (gamepad1.dpad_right)  {
            ShooterConstants.turretOffset(-40);
            sleep(100);
        }
        if (gamepad1.dpad_left) {
            ShooterConstants.turretOffset(40);
            sleep(100);
        }
        if (gamepad1.a) {
            ShooterConstants.hoodOffset(0.1);
            new Delay(0.5);
        }
        if (gamepad1.b) {
            ShooterConstants.hoodOffset(-0.1);
            new Delay(0.5);
        }
        if (gamepad1.x) {
            ShooterConstants.flywheelOffset(100);
            sleep(100);
        }
        if (gamepad1.y) {
            ShooterConstants.flywheelOffset(-100);
            sleep(100);
        }
        if (gamepad1.start) {
            PedroComponent.follower().setPose(poseInicial);
        }

        Shooter.INSTANCE.periodic();
        Hood.INSTANCE.periodic();

        /*telemetry.addData("heading", poseAtual.getHeading());
        telemetry.addData("turretAngle", Turret.turretAngle);
        telemetry.addData("destinationAngle", Turret.destinationAngle);
        telemetry.addData("y", poseAtual.getY());
        telemetry.addData("x", poseAtual.getX());
        telemetry.addData("flywheelVelocity", Shooter.INSTANCE.getVelocity());
        telemetry.addData("leftStickY", Gamepads.gamepad1().leftStickY().get());
        telemetry.addData("leftStickX", Gamepads.gamepad1().leftStickX().get());
        telemetry.addData("rightStickX", Gamepads.gamepad1().rightStickX().get());*/
        telemetry.addData("velOffset", ShooterConstants.getFlywheelOffset());
        //  telemetry.addData("offset", offset);

        telemetry.update();
    }
}
package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.goalPose;
import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.last;
import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.poseInicial;
import static org.firstinspires.ftc.teamcode.Constants.PoseManager.currentPose;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.calculatedDestinationAngle;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.LimelightHelper;
import org.firstinspires.ftc.teamcode.Constants.TelemetryHelper;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lock;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

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
import dev.nextftc.hardware.impl.ServoEx;

@Configurable
@TeleOp(name = "TeleOpVermelho")
public class TeleOpVermelho extends NextFTCOpMode {
    public TeleOpVermelho() {
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
    private double lastAngleLL = 0;

    private DriverControlledCommand driverControlled;
    public static double goalx = 15;
    public static double goaly = 160;
    public static boolean debugMode = true;
    public static boolean align = false;

    public static double hood = 0;
    private final ServoEx servoHood = new ServoEx("sHood");

    FTCCoordinates ftccoords;

    @Override
    public void onInit() {
        Lock.INSTANCE.closed.invoke();
        angleLL = 0;
        PedroComponent.follower().setStartingPose(poseInicial.mirror());
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(400);
        limelight.pipelineSwitch(4);
        limelight.start();
        PedroComponent.follower().update();

    }

    @Override
    public void onStartButtonPressed() {
        PedroComponent.follower().setStartingPose(poseInicial.mirror());
        PedroComponent.follower().update();
        driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
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
        Gamepads.gamepad1().b().whenTrue(() -> {
            Intake.INSTANCE.reversed.schedule();
        });
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> {
            if (Math.abs(lastAngleLL) < 6) {
                new SequentialGroup(
                        Hood.INSTANCE.medio,
                        Lock.INSTANCE.open,
                        Intake.INSTANCE.intake,
                        new Delay(0.4),
                        //Hood.INSTANCE.medio,
                        new Delay(0.6),
                        Intake.INSTANCE.stop
                        //Hood.INSTANCE.baixo
                ).schedule();
            }else{
                Turret.addOffset(angleLL);
            }
        });
    }

    @Override
    public void onUpdate() {
        PedroComponent.follower().update();

        driverControlled.update();
        Pose poseAtual = PedroComponent.follower().poseTracker.getPose();

        angleLL = LimelightHelper.updateAngleLL(limelight);
        LLResult result = limelight.getLatestResult();
        telemetry.update();
        if ((!(PedroComponent.follower().getAngularVelocity() > 1))&& angleLL != 0.0) {
            Turret.INSTANCE.setPoseTracker(poseAtual.getX(), poseAtual.getY(), Math.toDegrees(poseAtual.getHeading()), angleLL, false, telemetry);
            lastAngleLL = angleLL;
        }else{
            Turret.INSTANCE.setPoseTracker(poseAtual.getX(), poseAtual.getY(), Math.toDegrees(poseAtual.getHeading()), lastAngleLL, false, telemetry);
        }
        Turret.INSTANCE.periodic();
        telemetry.addData("destination", calculatedDestinationAngle);

        double distanceToGoal = PedroComponent.follower().poseTracker.getPose().distanceFrom(goalPose);
        Shooter.INSTANCE.setGoalDistance(distanceToGoal);
        Hood.INSTANCE.periodic();

        TelemetryHelper.addCommonTelemetry(telemetry, poseAtual, Shooter.INSTANCE.getVelocity(),
                Turret.turretAngle, Turret.destinationAngle, Turret.INSTANCE.toTurn,
                limelight, angleLL, debugMode, distanceToGoal, 0, 0, 0);
        telemetry.update();
    }
}
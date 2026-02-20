package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.goalPoseazul;
import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.poseInicial;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.calculatedDestinationAngle;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.toTurn;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.turretAngle;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ftc.FTCCoordinates;
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

    public static boolean debugMode = true;



    @Override
    public void onInit() {
        Lock.INSTANCE.closed.invoke();
        angleLL = 0;
        PedroComponent.follower().setStartingPose(poseInicial.mirror());
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(40);
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
        Gamepads.gamepad1().x().whenTrue(() -> driverControlled.setScalar(0.5));

        Gamepads.gamepad1().x().whenFalse(() -> driverControlled.setScalar(1));

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
            if (Math.abs(lastAngleLL) < 6 || (gamepad1.a)) {
                new SequentialGroup(
                        Lock.INSTANCE.open,
                        Intake.INSTANCE.shooting,
                        new Delay(1),
                        Intake.INSTANCE.stop
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

        angleLL = -LimelightHelper.updateAngleLL(limelight);
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

        double distanceToGoal = PedroComponent.follower().poseTracker.getPose().distanceFrom(goalPoseazul);

        Shooter.INSTANCE.setGoalDistance(distanceToGoal);
        Shooter.INSTANCE.periodic();

        Hood.INSTANCE.setGoalDistance(poseAtual.distanceFrom(goalPoseazul.mirror()));
        Hood.INSTANCE.periodic();

        TelemetryHelper.addCommonTelemetry(telemetry, PedroComponent.follower().getPose(),
                Shooter.INSTANCE.getVelocity(), turretAngle, Turret.destinationAngle, toTurn,
                limelight, angleLL, debugMode, distanceToGoal, 0.0, 0.0, 0.0);
        telemetry.update();
    }
}
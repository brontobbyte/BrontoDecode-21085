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
    FTCCoordinates ftccoords;

    @Override
    public void onInit() {
        angleLL = 0;
        PedroComponent.follower().setStartingPose(currentPose);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(400);
        limelight.pipelineSwitch(4);
        limelight.start();
    }

    @Override
    public void onStartButtonPressed() {
        driverControlled = new PedroDriverControlled(
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
                    Intake.INSTANCE.intake,
                    new Delay(1),
                    Intake.INSTANCE.stop
            ).schedule();
        });
    }

    @Override
    public void onUpdate() {

        Pose poseAtual = PedroComponent.follower().poseTracker.getPose();
        if (LimelightHelper.getRobotPoseFromCamera(limelight, poseAtual.getHeading()) != null) {
            Pose pedroPose = LimelightHelper.getRobotPoseFromCamera(limelight, poseAtual.getHeading());
            assert pedroPose != null;
            Pose pedroRealPose = pedroPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
            telemetry.addData("x2", (pedroRealPose.getX()));
            telemetry.addData("y2", (pedroRealPose.getY()));
            //PedroComponent.follower().setPose(LimelightHelper.getRobotPoseFromCamera(limelight, poseAtual.getHeading()));
        }else{
            PedroComponent.follower().update();
        }
        angleLL = LimelightHelper.updateAngleLL(limelight);
        LLResult result = limelight.getLatestResult();
        telemetry.update();
        Turret.INSTANCE.setPoseTracker(poseAtual.getX(), poseAtual.getY(), Math.toDegrees(poseAtual.getHeading()), angleLL);
        Turret.INSTANCE.periodic();

        double distanceToGoal = PedroComponent.follower().poseTracker.getPose().distanceFrom(goalPose);
        Shooter.INSTANCE.setGoalDistance(distanceToGoal);
        Hood.INSTANCE.periodic();

        double leftStickY = Gamepads.gamepad1().leftStickY().get();
        double leftStickX = Gamepads.gamepad1().leftStickX().get();
        double rightStickX = Gamepads.gamepad1().rightStickX().get();

        TelemetryHelper.addCommonTelemetry(telemetry, poseAtual, Shooter.INSTANCE.getVelocity(),
                Turret.turretAngle, Turret.destinationAngle, Turret.INSTANCE.toTurn,
                limelight, angleLL, debugMode, distanceToGoal, leftStickY, leftStickX, rightStickX);
        telemetry.update();
    }
}
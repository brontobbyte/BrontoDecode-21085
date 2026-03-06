package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.goalPoseazul;
import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.goalShootPoseAzul;
import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.poseInicial;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.calculatedDestinationAngle;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.destinationAngle;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.toTurn;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.turretAngle;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.LimelightHelper;
import org.firstinspires.ftc.teamcode.Constants.PoseManager;
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
    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.75
    );
    private static final Style robotLook2 = new Style(
            "", "#800000", 0.75
    );
    private static final Style robotLook3 = new Style(
            "", "#008000", 0.75
    );
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
    private Limelight3A limelight;
    private double angleLL = 0;
    private double lastAngleLL = 0;
    private double compensationX = 0;
    private double compensationY = 0;
    private double shooter = 0;
    public static double offsetturret = 1;
    public static double compensation = 1.45;
    private DriverControlledCommand driverControlled;
    public static boolean debugMode = true;

    @Override
    public void onInit() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        Lock.INSTANCE.closed.invoke();
        angleLL = 0;
        Pose startPose;
        if (PoseManager.currentPose != null) {
            startPose = PoseManager.currentPose;
        } else {
            startPose = poseInicial.mirror();
        }
        PedroComponent.follower().setStartingPose(startPose);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(40);
        limelight.pipelineSwitch(5);
        limelight.start();
        PedroComponent.follower().update();
    }
    @Override
    public void onStartButtonPressed() {
        Pose poseAtual = PedroComponent.follower().poseTracker.getPose();
        Turret.INSTANCE.setPoseTracker(poseAtual.getX(), poseAtual.getY(), Math.toDegrees(poseAtual.getHeading()), angleLL, false, telemetry, 0);
        PedroComponent.follower().update();

        driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX(),
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
            new SequentialGroup(
                    Lock.INSTANCE.open,
                    Intake.INSTANCE.shooting,
                    new Delay(0.7),
                    Intake.INSTANCE.stop
            ).schedule();
        });
        Gamepads.gamepad1().x().whenBecomesTrue(this::resetPose);
        Gamepads.gamepad1().a().whenBecomesTrue(this::resetTurretPose);
    }
    private void resetPose() {
        PedroComponent.follower().setPose(poseInicial.mirror());
        PedroComponent.follower().update();
    }
    private void resetTurretPose() {
        Turret.INSTANCE.resetTurret();
    }
    @Override
    public void onUpdate() {
        PedroComponent.follower().update();
        driverControlled.update();

        Pose poseAtual = PedroComponent.follower().poseTracker.getPose();
        Pose poseTurret = new Pose(0, 0, Math.toRadians(calculatedDestinationAngle));
        Pose poseTurret2 = new Pose(0, 0, Math.toRadians(destinationAngle));
        Pose poseTurretAtual = new Pose(0, 0, Math.toRadians(toTurn));
        angleLL = -LimelightHelper.updateAngleLL(limelight);
        double xVelo = PedroComponent.follower().getVelocity().getXComponent();
        double yVelo = PedroComponent.follower().getVelocity().getYComponent();

        if (Math.abs(xVelo) < -10){
            compensationX = xVelo/4;
        }else{
            compensationX = 0;
        }
        if (Math.abs(yVelo) > 10){
            compensationY = yVelo/compensation;
        }else{
            compensationY = 0;
        }
        if ((!(PedroComponent.follower().getAngularVelocity() > 1)) && angleLL != 0.0) {
            Turret.INSTANCE.setPoseTracker(poseAtual.getX(), poseAtual.getY(), Math.toDegrees(poseAtual.getHeading()), angleLL + offsetturret, false, telemetry, compensationX + compensationY);
            lastAngleLL = angleLL;
        } else {
            Turret.INSTANCE.setPoseTracker(poseAtual.getX(), poseAtual.getY(), Math.
                    toDegrees(poseAtual.getHeading()), lastAngleLL, false, telemetry, compensationX + compensationY);
        }
        Turret.INSTANCE.periodic();
        double distanceToGoal = PedroComponent.follower().poseTracker.getPose().distanceFrom(goalShootPoseAzul.mirror());

        panelsField.setStyle(robotLook);
        panelsField.moveCursor(poseAtual.getX(), poseAtual.getY());
        panelsField.circle(9);
        Vector v = poseTurret.getHeadingAsUnitVector();
        Vector v2 = poseTurret2.getHeadingAsUnitVector();
        Vector v3 = poseTurretAtual.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * 9);
        double x1 = poseAtual.getX() + v.getXComponent() / 2, y1 = poseAtual.getY() + v.getYComponent() / 2;
        double x2 = poseAtual.getX() + v.getXComponent(), y2 = poseAtual.getY() + v.getYComponent();
        panelsField.setStyle(robotLook);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
        v2.setMagnitude(v2.getMagnitude() * 9);
        double x3 = poseAtual.getX() + v2.getXComponent() / 2, y3 = poseAtual.getY() + v2.getYComponent() / 2;
        double x4 = poseAtual.getX() + v2.getXComponent(), y4 = poseAtual.getY() + v2.getYComponent();
        panelsField.setStyle(robotLook2);
        panelsField.moveCursor(x3, y3);
        panelsField.line(x4, y4);
        v3.setMagnitude(v3.getMagnitude() * 9);
        double x5 = poseAtual.getX() + v3.getXComponent() / 2, y5 = poseAtual.getY() + v3.getYComponent() / 2;
        double x6 = poseAtual.getX() + v3.getXComponent(), y6 = poseAtual.getY() + v3.getYComponent();
        panelsField.setStyle(robotLook3);
        panelsField.moveCursor(x5, y5);
        panelsField.line(x6, y6);
        panelsField.update();

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
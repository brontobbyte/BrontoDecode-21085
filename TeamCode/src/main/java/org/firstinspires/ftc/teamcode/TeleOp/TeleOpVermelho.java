package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.goalShootPoseVermelho;
import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.poseInicial;
import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.poseResetHumanPVemrelho;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.destinationAngle;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.toTurn;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.turretAngle;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants.PoseManager;
import org.firstinspires.ftc.teamcode.Constants.TelemetryHelper;
import org.firstinspires.ftc.teamcode.Constants.autoshoot;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lock;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Configurable
@TeleOp(name = "TeleOpVermelho")
public class TeleOpVermelho extends NextFTCOpMode {

    public TeleOpVermelho() {
        addComponents(
                new SubsystemComponent(Shooter.INSTANCE),
                new SubsystemComponent(Hood.INSTANCE),
                new SubsystemComponent(Turret.INSTANCE),
                new SubsystemComponent(Intake.INSTANCE),
                new SubsystemComponent(Lock.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private static final FieldManager panelsField =
            PanelsField.INSTANCE.getField();

    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;

    private static final Style robotLook  = new Style("", "#3F51B5", 0.75);
    private static final Style robotLook2 = new Style("", "#800000", 0.75);
    private static final Style robotLook3 = new Style("", "#008000", 0.75);

    public static double compensation          = 0;
    public static double flywheelCompensation  = 0.75;
    public static double projectileSpeed       = 210;
    public static double projectileSpeedAutoY  = 4000;
    public static double projectileSpeedAutoX  = 322;
    public static boolean debugMode            = true;
    public static double targetHeadingDeg      = 149;
    public static double kpHeading             = 0.5;
    public static double max                   = 0.9;
    public static double odometrycorrectiondeg = -0.41;

    private long   tempoloopanterior = 0;
    private double tempoloopmedio    = 0;
    private int    contagemLoops     = 0;
    private static final int medialooptime = 20;

    boolean lastDpadUp    = false;
    boolean lastb         = false;
    boolean lastDpadDown  = false;
    boolean lastDpadLeft  = false;
    boolean lastDpadRight = false;
    boolean lastx         = false;
    double  valorTurret   = 0;
    double  valorFlywheel = 0;

    @Override
    public void onInit() {
        Shooter.INSTANCE.setVelocity(0);
        Intake.INSTANCE.stop.invoke();

        FrontLeft  = hardwareMap.get(DcMotor.class, "fl");
        FrontRight = hardwareMap.get(DcMotor.class, "fr");
        BackLeft   = hardwareMap.get(DcMotor.class, "bl");
        BackRight  = hardwareMap.get(DcMotor.class, "br");

        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        panelsField.setOffsets(
                PanelsField.INSTANCE.getPresets().getPEDRO_PATHING()
        );

        try {
            PedroComponent.follower().getPoseTracker().getLocalizer().resetIMU();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        Pose startPose = (PoseManager.currentPose != null)
                ? PoseManager.currentPose
                : poseInicial;

        PedroComponent.follower().setStartingPose(startPose);
    }

    @Override
    public void onStartButtonPressed() {
        Pose poseAtual = PedroComponent.follower().getPose();
        Lock.INSTANCE.open.invoke();
        Turret.enabled = true;

        double correctionRad = Math.toRadians(odometrycorrectiondeg);

        double correctedX =
                poseAtual.getX() * Math.cos(correctionRad)
                        - poseAtual.getY() * Math.sin(correctionRad);

        double correctedY =
                poseAtual.getX() * Math.sin(correctionRad)
                        + poseAtual.getY() * Math.cos(correctionRad);

        poseAtual = new Pose(
                correctedX,
                correctedY,
                poseAtual.getHeading()
        );

        Turret.INSTANCE.setPoseTracker(
                poseAtual.getX(),
                poseAtual.getY(),
                poseAtual.getHeading(),
                0,
                false,
                telemetry,
                0
        );
        Lock.INSTANCE.closed.invoke();
        Intake.INSTANCE.intake.invoke();
        Indexer.INSTANCE.naoshooting.invoke();

        Shooter.INSTANCE.setVelocity(Shooter.goal);

        Gamepads.gamepad1().y().whenTrue(() ->
                Intake.INSTANCE.reversed.schedule()
        );
        Gamepads.gamepad1().a().whenTrue(() ->
                Intake.INSTANCE.intake.schedule()
        );

        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() ->
                new SequentialGroup(
                        Indexer.INSTANCE.shooting,
                        Lock.INSTANCE.open,
                        new Delay(0.7),
                        Lock.INSTANCE.closed,
                        Indexer.INSTANCE.naoshooting
                ).schedule()
        );

        Gamepads.gamepad1().x().whenBecomesTrue(this::resetPose);
    }

    @Override
    public void onUpdate() {

        long   tempoLoopAtual = System.nanoTime();
        double deltaNano      = tempoLoopAtual - tempoloopanterior;
        double tempoLoopMs    = deltaNano / 1_000_000.0;

        contagemLoops++;
        if (contagemLoops == 1) {
            tempoloopmedio = tempoLoopMs;
        } else {
            tempoloopmedio = ((tempoloopmedio * (contagemLoops - 1)) + tempoLoopMs) / contagemLoops;
        }
        if (contagemLoops >= medialooptime) {
            contagemLoops = 0;
        }
        tempoloopanterior = tempoLoopAtual;

        PedroComponent.follower().update();

        double y  = -gamepad1.left_stick_y;
        double x  = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double headingTurret = PedroComponent.follower().getPose().getHeading();

        if (gamepad1.left_bumper) {
            double targetRad = Math.toRadians(targetHeadingDeg + 180);
            double error = Math.atan2(
                    Math.sin(targetRad - headingTurret),
                    Math.cos(targetRad - headingTurret)
            );
            double errorDeg = Math.abs(Math.toDegrees(error));

            if (errorDeg > 45) {
                x *= 0.4;
                y *= 0.4;
            } else if (errorDeg > 25) {
                x *= 0.6;
                y *= 0.6;
            } else if (errorDeg > 10) {
                x *= 0.8;
                y *= 0.8;
            }

            rx = Math.max(-max, Math.min(max, error * kpHeading));
            if (errorDeg < 2.0) rx = 0;
        }

        double rotX = x * Math.cos(-headingTurret) - y * Math.sin(-headingTurret);
        double rotY = x * Math.sin(-headingTurret) + y * Math.cos(-headingTurret);
        rotX *= 1.1;

        double denominator = Math.max(
                Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0
        );

        FrontLeft.setPower((rotY + rotX + rx) / denominator);
        BackLeft.setPower((rotY - rotX + rx) / denominator);
        FrontRight.setPower((rotY - rotX - rx) / denominator);
        BackRight.setPower((rotY + rotX - rx) / denominator);

        Pose poseAtual       = PedroComponent.follower().getPose();
        double distanceToGoal = poseAtual.distanceFrom(goalShootPoseVermelho);

        Vector v = new Pose(0, 0, Math.toRadians(poseAtual.getHeading()))
                .getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * 9);

        Vector v2 = new Pose(0, 0, Math.toRadians(destinationAngle))
                .getHeadingAsUnitVector();
        v2.setMagnitude(v2.getMagnitude() * 9);

        Vector v3 = new Pose(0, 0, Math.toRadians(toTurn))
                .getHeadingAsUnitVector();
        v3.setMagnitude(v3.getMagnitude() * 9);

        panelsField.setStyle(robotLook);
        panelsField.moveCursor(poseAtual.getX() - 7.5175, poseAtual.getY() - 6.04);
        panelsField.rect(15.3543, 12.08661);

        panelsField.setStyle(robotLook);
        panelsField.moveCursor(
                poseAtual.getX() + v.getXComponent() / 2,
                poseAtual.getY() + v.getYComponent() / 2
        );
        panelsField.line(
                poseAtual.getX() + v.getXComponent(),
                poseAtual.getY() + v.getYComponent()
        );

        panelsField.setStyle(robotLook2);
        panelsField.moveCursor(
                poseAtual.getX() + v2.getXComponent() / 2,
                poseAtual.getY() + v2.getYComponent() / 2
        );
        panelsField.line(
                poseAtual.getX() + v2.getXComponent(),
                poseAtual.getY() + v2.getYComponent()
        );

        panelsField.setStyle(robotLook3);
        panelsField.moveCursor(
                poseAtual.getX() + v3.getXComponent() / 2,
                poseAtual.getY() + v3.getYComponent()
        );
        panelsField.line(
                poseAtual.getX() + v3.getXComponent(),
                poseAtual.getY() + v3.getYComponent()
        );

        panelsField.update();

        double lateralVelY  = PedroComponent.follower().poseTracker.getVelocity().getYComponent();
        double VerticalVelX = PedroComponent.follower().poseTracker.getVelocity().getXComponent();

        double flightTime = distanceToGoal / projectileSpeed;

        if (gamepad2.dpad_up && !lastDpadUp) {
            valorFlywheel += 40;
        }
        if (gamepad2.dpad_down && !lastDpadDown) {
            valorFlywheel -= 40;
        }
        if (gamepad2.dpad_right && !lastDpadRight) {
            valorTurret++;
        }
        if (gamepad2.dpad_left && !lastDpadLeft) {
            valorTurret--;
        }
        if (gamepad2.b && !lastb) {
            valorFlywheel = 0;
        }
        if (gamepad2.x && !lastx) {
            valorTurret = 0;
        }

        lastDpadUp    = gamepad2.dpad_up;
        lastDpadDown  = gamepad2.dpad_down;
        lastDpadLeft  = gamepad2.dpad_left;
        lastDpadRight = gamepad2.dpad_right;
        lastb         = gamepad2.b;
        lastx         = gamepad2.x;

        double leadCompensation = lateralVelY * flightTime + valorTurret;

        autoshoot.SetFlywheelOffset(valorFlywheel);

        Turret.INSTANCE.setPoseTracker(
                poseAtual.getX(),
                poseAtual.getY(),
                poseAtual.getHeading(),
                0.0,
                false,
                telemetry,
                leadCompensation
        );

        Shooter.INSTANCE.setGoalDistanceWithComp(distanceToGoal);
        Hood.INSTANCE.setGoalDistance(distanceToGoal);

        TelemetryHelper.addCommonTelemetry(
                telemetry,
                poseAtual,
                Shooter.INSTANCE.getVelocity(),
                turretAngle,
                destinationAngle,
                toTurn,
                null,
                0.0,
                debugMode,
                distanceToGoal,
                0.0,
                0.0,
                0.0
        );

        telemetry.addData("cos",           Math.cos(Math.toDegrees(poseAtual.getHeading())));
        telemetry.addData("heading graus", Math.toDegrees(poseAtual.getHeading()));
        telemetry.addData("goalx",         Turret.goalx);
        telemetry.addData("goaly",         Turret.goaly);
        telemetry.addData("toTurn",        toTurn);
        telemetry.addData("turretAngle",   turretAngle);

        telemetry.update();
    }
    @Override
    public void onStop() {
        Turret.enabled = false;
    }

    private void resetPose() {
        PedroComponent.follower().setPose(poseResetHumanPVemrelho);
    }
    private void resetTurretPose() {
        Turret.INSTANCE.resetTurret();
    }
}
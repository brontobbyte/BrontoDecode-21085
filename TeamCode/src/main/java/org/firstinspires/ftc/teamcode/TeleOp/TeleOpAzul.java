package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.goalShootPoseAzul;
import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.last;
import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.poseInicial;

import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.poseResetHumanPAzul;
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
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer;
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
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Configurable
@TeleOp(name = "TeleOpAzul")
public class TeleOpAzul extends NextFTCOpMode {

    public TeleOpAzul() {
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
    private static final Style robotLook = new Style("", "#3F51B5", 0.75);
    private static final Style robotLook2 = new Style("", "#800000", 0.75);
    private static final Style robotLook3 = new Style("", "#008000", 0.75);

    public static double compensation = 0;
    public static double flywheelCompensation = 0.75;

    public static double projectileSpeed = 80;
    public static boolean debugMode = true;
    public static double targetHeadingDeg = 149;
    public static double kpHeading = 0.5;
    public static double max = 0.9;

    private long tempoloopanterior = 0;
    private double tempoloopmedio = 0;
    private int contagemLoops = 0;
    private static final int medialooptime = 20;

    @Override
    public void onInit() {
        FrontLeft = hardwareMap.get(DcMotor.class, "fl");
        FrontRight = hardwareMap.get(DcMotor.class, "fr");
        BackLeft = hardwareMap.get(DcMotor.class, "bl");
        BackRight = hardwareMap.get(DcMotor.class, "br");

        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        panelsField.setOffsets(
                PanelsField.INSTANCE.getPresets().getPEDRO_PATHING()
        );

        Lock.INSTANCE.closed.invoke();

        Pose startPose =
                (PoseManager.currentPose != null)
                        ? PoseManager.currentPose
                        : poseInicial;

        PedroComponent.follower().setStartingPose(startPose);

        tempoloopanterior = System.nanoTime();
    }

    @Override
    public void onStartButtonPressed() {
        Pose poseAtual = PedroComponent.follower().getPose();

        Turret.INSTANCE.setPoseTracker(
                poseAtual.getX(),
                poseAtual.getY(),
                poseAtual.getHeading(),
                0.0,
                true,
                telemetry,
                0.0
        );

        Lock.INSTANCE.closed.invoke();
//        Intake.INSTANCE.intake.invoke();
        Indexer.INSTANCE.naoshooting.invoke();

        Shooter.INSTANCE.setVelocity(Shooter.goal);

        Gamepads.gamepad1().y().whenTrue(() ->
                Intake.INSTANCE.reversed.schedule()
        );

        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() ->
                new SequentialGroup(
//                        Intake.INSTANCE.intake,
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

        long tempoLoopAtual = System.nanoTime();
        double deltaNano = tempoLoopAtual - tempoloopanterior;
        double tempoLoopMs = deltaNano / 1_000_000.0;

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

        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
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

        Pose poseAtual = PedroComponent.follower().getPose();
        double distanceToGoal = poseAtual.distanceFrom(goalShootPoseAzul);

        Vector v = new Pose(
                0,
                0,
                Math.toRadians(poseAtual.getHeading())
        ).getHeadingAsUnitVector();

        v.setMagnitude(v.getMagnitude() * 9);

        Vector v2 = new Pose(
                0,
                0,
                Math.toRadians(destinationAngle)
        ).getHeadingAsUnitVector();

        v2.setMagnitude(v2.getMagnitude() * 9);

        Vector v3 = new Pose(
                0,
                0,
                Math.toRadians(toTurn)
        ).getHeadingAsUnitVector();

        v3.setMagnitude(v3.getMagnitude() * 9);

        panelsField.setStyle(robotLook);
        panelsField.moveCursor(poseAtual.getX()-7.5175, poseAtual.getY()-6.04);
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

//        double dx   = Turret.goalx - poseAtual.getX();
//        double dy   = Turret.goaly - poseAtual.getY();
//        double dist = Math.sqrt(dx * dx + dy * dy);
//
//        double vx = PedroComponent.follower().getVelocity().getXComponent();
//        double vy = PedroComponent.follower().getVelocity().getYComponent();
//
//        double radialVelX = dx / dist;
//        double radialVelY = dy / dist;
//        double lateralVel = vx * (-radialVelY) + vy * radialVelX;
        double lateralVel = PedroComponent.follower().poseTracker.getVelocity().getYComponent();

        double flightTime = distanceToGoal / projectileSpeed;

        double leadCompensation = lateralVel * flightTime;

        Turret.INSTANCE.setPoseTracker(
                poseAtual.getX(),
                poseAtual.getY(),
                poseAtual.getHeading(),
                0.0,
                true,
                telemetry,
                leadCompensation
        );

//        double radialVel = (vx * dx + vy * dy) / dist;
//        Shooter.radialCompensation = radialVel * flywheelCompensation;

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

        telemetry.update();
    }

    private void resetPose() {
        PedroComponent.follower().setPose(poseResetHumanPAzul);
    }

    private void resetTurretPose() {
        Turret.INSTANCE.resetTurret();
    }
}
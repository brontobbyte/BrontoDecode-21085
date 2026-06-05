package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.goalPoseazul;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lock;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Configurable
@TeleOp(name = "FlywheelExample")
public class FlywheelExample extends NextFTCOpMode {

    {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE),
                new SubsystemComponent(Shooter.INSTANCE),
                new SubsystemComponent(Hood.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    public static double hood = 0;
    public static double intake = 1;

    public static double poselegalimportantex = 8.612928348909657;
    public static double poselegalimportantey = 8.220404984423668;

    public static boolean redAlliance = false;

    private final ServoEx servoHood = new ServoEx("sHood");
    private final MotorEx intakeMotor = new MotorEx("intake").reversed();

    private boolean shooterOn = false;
    private boolean lastAState = false;

    private double lastFkp = Shooter.Fkp;
    private double lastFki = Shooter.Fki;
    private double lastFkd = Shooter.Fkd;
    private double lastFkv = Shooter.Fkv;
    private double lastFka = Shooter.Fka;
    private double lastFks = Shooter.Fks;

    private Pose pose(double x, double y, double headingDegrees) {
        Pose p = new Pose(x, y, Math.toRadians(headingDegrees));
        return redAlliance ? p.mirror() : p;
    }

    @Override
    public void onInit() {
        PedroComponent.follower().setStartingPose(
                pose(poselegalimportantex, poselegalimportantey, 90)
        );

        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() ->
                new SequentialGroup(
                        Lock.INSTANCE.open,
                        Indexer.INSTANCE.shooting,
                        Intake.INSTANCE.shooting,
                        new Delay(0.7),
                        Intake.INSTANCE.stop
                ).schedule()
        );
    }

    @Override
    public void onUpdate() {

        if (gamepad1.a && !lastAState) {
            shooterOn = !shooterOn;
            if (!shooterOn) {
                Shooter.INSTANCE.stop();
            }
        }
        lastAState = gamepad1.a;

        if (gamepad1.b) {
            intakeMotor.setPower(-intake);
            Lock.INSTANCE.closed.schedule();
            Indexer.INSTANCE.naoshooting.schedule();
        } else {
            intakeMotor.setPower(0);
        }

        servoHood.setPosition(hood);

        if (shooterOn) {
            if (gainsTuned()) {
                Shooter.INSTANCE.rebuildControlSystem();
                syncGainCache();
            }
            Shooter.INSTANCE.setVelocity(Shooter.goal);
        }

        PedroComponent.follower().update();

        Pose goalPose = redAlliance ? goalPoseazul.mirror() : goalPoseazul;
        double dist = PedroComponent.follower().poseTracker.getPose().distanceFrom(goalPose);

        telemetry.addLine("=== ALIANÇA ===");
        telemetry.addData("Alliance", redAlliance ? "VERMELHO" : "AZUL");

        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Shooter On", shooterOn);
        telemetry.addData("Target", Shooter.goal);
        telemetry.addData("Velocity", Shooter.INSTANCE.getVelocity());
        telemetry.addData("Error", Shooter.INSTANCE.getVelocityError());
        telemetry.addData("Power", Shooter.INSTANCE.getPower());

        telemetry.addLine("=== GAINS ===");
        telemetry.addData("Fkp", Shooter.Fkp);
        telemetry.addData("Fki", Shooter.Fki);
        telemetry.addData("Fkd", Shooter.Fkd);

        telemetry.addLine("=== FEEDFORWARD ===");
        telemetry.addData("Fkv", Shooter.Fkv);
        telemetry.addData("Fka", Shooter.Fka);
        telemetry.addData("Fks", Shooter.Fks);

        telemetry.addLine("=== POSE ===");
        telemetry.addData("x", PedroComponent.follower().poseTracker.getPose().getX());
        telemetry.addData("y", PedroComponent.follower().poseTracker.getPose().getY());
        telemetry.addData("dist", dist);

        telemetry.update();
    }

    private boolean gainsTuned() {
        return Shooter.Fkp != lastFkp
                || Shooter.Fki != lastFki
                || Shooter.Fkd != lastFkd
                || Shooter.Fkv != lastFkv
                || Shooter.Fka != lastFka
                || Shooter.Fks != lastFks;
    }

    private void syncGainCache() {
        lastFkp = Shooter.Fkp;
        lastFki = Shooter.Fki;
        lastFkd = Shooter.Fkd;
        lastFkv = Shooter.Fkv;
        lastFka = Shooter.Fka;
        lastFks = Shooter.Fks;
    }
}
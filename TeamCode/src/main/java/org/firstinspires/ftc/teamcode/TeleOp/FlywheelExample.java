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

    public static double poselegalimportantex = 26.96435342014482;
    public static double poselegalimportantey = 126.87326351120366;

    public static boolean redAlliance = false;

    private final ServoEx servoHood = new ServoEx("sHood");
    private final MotorEx intakeMotor = new MotorEx("intake").reversed();

    private boolean shooterOn = false;
    private boolean lastAState = false;

    private Pose pose(double x, double y, double headingDegrees) {
        Pose p = new Pose(x, y, Math.toRadians(headingDegrees));
        return redAlliance ? p.mirror() : p;
    }

    @Override
    public void onInit() {
        PedroComponent.follower().setStartingPose(
                pose(poselegalimportantex, poselegalimportantey, 90)
        );
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> {

            new SequentialGroup(
                    Lock.INSTANCE.open,
                    Indexer.INSTANCE.shooting,
                    Intake.INSTANCE.shooting,
                    new Delay(0.7),
                    Intake.INSTANCE.stop
            ).schedule();

        });
    }

    @Override
    public void onUpdate() {

        if (gamepad1.a && !lastAState) {
            shooterOn = !shooterOn;
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
            Shooter.INSTANCE.setVelocity(Shooter.goal);
        } else {
            Shooter.INSTANCE.setSpeed(0);
        }

        PedroComponent.follower().update();

        Pose goalPose = redAlliance ? goalPoseazul.mirror() : goalPoseazul;

        telemetry.addLine("Aliança");
        telemetry.addData("Alliance", redAlliance ? "VERMELHO" : "AZUL");

        telemetry.addLine("Shooter");
        telemetry.addData("Shooter On", shooterOn);

        telemetry.addLine("PID");
        telemetry.addData("Fkp", Shooter.Fkp);
        telemetry.addData("Fki", Shooter.Fki);
        telemetry.addData("Fkd", Shooter.Fkd);

        telemetry.addLine("FeedForward");
        telemetry.addData("Fks", Shooter.Fks);
        telemetry.addData("Fka", Shooter.Fka);
        telemetry.addData("Fkv", Shooter.Fkv);

        telemetry.addLine("Goal");
        telemetry.addData("Goal Velocity", Shooter.goal);

        telemetry.addLine("Motor");
        telemetry.addData("Velocity", Shooter.INSTANCE.getVelocity());
        telemetry.addData("Power", Shooter.INSTANCE.getPower());

        telemetry.addLine("Pose");
        telemetry.addData(
                "dist",
                PedroComponent.follower()
                        .poseTracker
                        .getPose()
                        .distanceFrom(goalPose)
        );

        telemetry.addData(
                "x",
                PedroComponent.follower()
                        .poseTracker
                        .getPose()
                        .getX()
        );

        telemetry.addData(
                "y",
                PedroComponent.follower()
                        .poseTracker
                        .getPose()
                        .getY()
        );

        telemetry.update();
    }
}
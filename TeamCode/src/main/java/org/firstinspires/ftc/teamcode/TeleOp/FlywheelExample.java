package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.goalPoseazul;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lock;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.core.commands.groups.SequentialGroup;

@Configurable
@TeleOp
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

    private ControlSystem controller;

    public static double Fkp = 0.000001;
    public static double Fki = 0.0000000000099;
    public static double Fkd = 0.002;

    public static double Fks = 0.15;
    public static double Fka = 0.7;
    public static double Fkv = 0.00034;

    public static double goal = 700;
    public static double hood = 0;
    public static double intake = 1;

    public static double poselegalimportantex = 118;
    public static double poselegalimportantey = 118;

    public DcMotorEx flywheelMotor1;
    public DcMotorEx flywheelMotor2;

    private final ServoEx servoHood = new ServoEx("sHood");
    private MotorEx motor = new MotorEx("intake").reversed();

    private boolean shooterOn = false;
    private boolean lastAState = false;

    @Override
    public void onInit() {

        PedroComponent.follower().setStartingPose(
                new Pose(poselegalimportantex, poselegalimportantey, Math.toRadians(90))
        );

        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "f1");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "f2");

        rebuildController();

        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> {
            new SequentialGroup(
                    Intake.INSTANCE.shooting,
                    new Delay(0.7),
                    Intake.INSTANCE.stop
            ).schedule();
        });
    }

    private void rebuildController() {
        controller = ControlSystem.builder()
                .velPid(Fkp, Fki, Fkd)
                .basicFF(Fkv, Fka, Fks)
                .build();

        controller.setGoal(new KineticState(0.0, goal));
    }

    @Override
    public void onUpdate() {

        if (gamepad1.a && !lastAState) {
            shooterOn = !shooterOn;
        }
        lastAState = gamepad1.a;

        if (gamepad1.b) {
            motor.setPower(intake);
        } else {
            motor.setPower(0);
        }

        rebuildController();

        servoHood.setPosition(hood);

        double power = 0;
        double power2 = 0;

        if (shooterOn) {

            power = controller.calculate(new KineticState(
                    flywheelMotor1.getCurrentPosition(),
                    flywheelMotor1.getVelocity()));

            power2 = controller.calculate(new KineticState(
                    flywheelMotor2.getCurrentPosition(),
                    flywheelMotor2.getVelocity()));
        }

        flywheelMotor1.setPower(power);
        flywheelMotor2.setPower(power2);

        PedroComponent.follower().update();

        telemetry.addLine("shooter");
        telemetry.addData("Shooter On", shooterOn);

        telemetry.addLine("pid");
        telemetry.addData("Fkp", Fkp);
        telemetry.addData("Fki", Fki);
        telemetry.addData("Fkd", Fkd);

        telemetry.addLine("feed");
        telemetry.addData("Fks", Fks);
        telemetry.addData("Fka", Fka);
        telemetry.addData("Fkv", Fkv);

        telemetry.addLine("goal");
        telemetry.addData("Goal Velocity", goal);

        telemetry.addLine("motor");
        telemetry.addData("Velocity1", flywheelMotor1.getVelocity());
        telemetry.addData("Velocity2", flywheelMotor2.getVelocity());

        telemetry.addLine("saida");
        telemetry.addData("Power1", power);
        telemetry.addData("Power2", power2);
        telemetry.addData("Shooter On", shooterOn);
        telemetry.addData("velo1", flywheelMotor1.getVelocity());
        telemetry.addData("velo2", flywheelMotor2.getVelocity());
        telemetry.addData("dist",
                PedroComponent.follower().poseTracker.getPose()
                        .distanceFrom(goalPoseazul.mirror()));
        telemetry.addData("x",
                PedroComponent.follower().poseTracker.getPose().getX());
        telemetry.addData("y",
                PedroComponent.follower().poseTracker.getPose().getY());
        telemetry.update();
        telemetry.update();
    }
}


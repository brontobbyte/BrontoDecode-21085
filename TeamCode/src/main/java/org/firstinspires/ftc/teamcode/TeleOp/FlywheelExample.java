package org.firstinspires.ftc.teamcode.TeleOp;


import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.goalPose;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lock;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Configurable
@TeleOp
public class FlywheelExample extends NextFTCOpMode {

    {
        addComponents(
                new PedroComponent(Constants::createFollower)
        );
    }

    private ControlSystem controller;

    public static double Fkp = 0.00099;
    public static double Fki = 0.00000000001;
    public static double Fkd = 0.00001;
    public static double Fks = 0.3;
    public static double Fka = 2;
    public static double Fkv = 0.000236;
    public static double goal = 500;
    public static double hood = 0.28;
    public static double poselegalimportantex = 60;
    public static double poselegalimportantey = 11.77981651376144;

    public DcMotorEx flywheelMotor1;
    public DcMotorEx flywheelMotor2;


    private final ServoEx servoHood = new ServoEx("sHood");
    private MotorEx motor = new MotorEx("intake");

    @Override
    public void onInit() {
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "f1");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "f2");
    }

    @Override
    public void onUpdate() {
        if (gamepad1.a) {
            motor.setPower(1);
        } else {
            motor.setPower(0);
        }
        controller = ControlSystem.builder()
                .velPid(Fkp, Fki, Fkd)
                .basicFF(Fkv, Fka, Fks)
                .build();

        controller.setGoal(new KineticState(0.0, goal));
        servoHood.setPosition(hood);
        flywheelMotor1.setPower(controller.calculate(new KineticState(
                flywheelMotor1.getCurrentPosition(),
                flywheelMotor1.getVelocity()))
        );
        flywheelMotor2.setPower(controller.calculate(new KineticState(
                flywheelMotor2.getCurrentPosition(),
                flywheelMotor2.getVelocity()))
        );
        PedroComponent.follower().setStartingPose(new Pose(poselegalimportantex, poselegalimportantey,Math.toRadians(180)));
        PedroComponent.follower().update();
        telemetry.addData("velo", flywheelMotor2.getVelocity());
        telemetry.addData("velo2", flywheelMotor1.getVelocity());
        telemetry.addData("dist", PedroComponent.follower().poseTracker.getPose().distanceFrom(goalPose));
        telemetry.addData("x", PedroComponent.follower().poseTracker.getPose().getX());
        telemetry.addData("y", PedroComponent.follower().poseTracker.getPose().getY());


        telemetry.update();
    }
}
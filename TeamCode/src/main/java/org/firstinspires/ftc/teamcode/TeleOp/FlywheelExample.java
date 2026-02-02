package org.firstinspires.ftc.teamcode.TeleOp;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Configurable
@TeleOp
public class FlywheelExample extends OpMode {


    private ControlSystem controller;

    public static double Fkp = 0.0001;
    public static double Fki = 0.000000000001;
    public static double Fkd = 0.00001;
    public static double Fks = 0.3;
    public static double Fka = 1;
    public static double Fkv = 0.0003789;
    public static double goal = 1000;
    public static double hood = 0.18;
    public DcMotorEx flywheelMotor1;
    public DcMotorEx flywheelMotor2;


    private final ServoEx servoHood = new ServoEx("sHood");
    private MotorEx motor = new MotorEx("intake");

    @Override
    public void init() {
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "f1");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "f2");
    }

    @Override
    public void loop() {
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
        telemetry.addData("velo", flywheelMotor1.getVelocity());
        telemetry.update();
    }
}
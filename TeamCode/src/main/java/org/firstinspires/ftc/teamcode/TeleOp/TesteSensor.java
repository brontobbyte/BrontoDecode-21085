package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.SensorColorLED;

@TeleOp
public class TesteSensor extends LinearOpMode {

    NormalizedColorSensor s1, s2, s3;
    Servo led;
    DcMotor intake;

    @Override
    public void runOpMode() {

        s1 = hardwareMap.get(NormalizedColorSensor.class, "sensorcor1");
        s2 = hardwareMap.get(NormalizedColorSensor.class, "sensorcor2");
        s3 = hardwareMap.get(NormalizedColorSensor.class, "sensorcor3");

        led = hardwareMap.get(Servo.class, "led");
        intake = hardwareMap.get(DcMotor.class, "intake");

        waitForStart();

        SensorColorLED.startThread(s1, s2, s3, led);

        while (opModeIsActive()) {

            if (SensorColorLED.count < 3) {
                intake.setPower(1.0);
            } else {
                intake.setPower(0);
            }

            telemetry.addData("A1", SensorColorLED.a1);
            telemetry.addData("A2", SensorColorLED.a2);
            telemetry.addData("A3", SensorColorLED.a3);
            telemetry.addData("COUNT", SensorColorLED.count);
            telemetry.update();
        }

        SensorColorLED.stop();
    }
}
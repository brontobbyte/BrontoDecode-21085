package org.firstinspires.ftc.teamcode.Programs;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@TeleOp(name = "Turret + Field Oriented", group = "turret")
public class fieldtagjoao extends LinearOpMode {
    private DcMotorEx motorTurret;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;
    private final double GEAR_RATIO = 1.0;
    private final double tickspor360 = 998.0 * GEAR_RATIO;
    private final double kP = 0.05;

    private double headingInicial = 0;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                ));
        imu.initialize(parameters);

        motorTurret = hardwareMap.get(DcMotorEx.class, "motor_turret");
        motorTurret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorTurret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorTurret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft = hardwareMap.get(DcMotorEx.class, "motor_esquerda");
        frontRight = hardwareMap.get(DcMotorEx.class, "motor_direita");
        backLeft = hardwareMap.get(DcMotorEx.class, "motor_esquerdatras");
        backRight = hardwareMap.get(DcMotorEx.class, "motor_direitatras");

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        telemetry.addLine("nada");
        telemetry.update();

        waitForStart();

        headingInicial = getHeading();

        while (opModeIsActive()) {
            double headingAtual = getHeading();

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double botHeading = Math.toRadians(getHeading());

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            double deltaHeading = angleWrap(headingAtual - headingInicial);

            double setpointGraus = -deltaHeading;

            double posicaoAtualGraus = (motorTurret.getCurrentPosition() / tickspor360) * 360.0;

            double erro = angleWrap(setpointGraus - posicaoAtualGraus);
            double saida = Range.clip(kP * erro, -0.7, 0.7);
            motorTurret.setPower(saida);

            telemetry.addLine("turret");

            telemetry.addData("Heading Inicial", "%.2f", headingInicial);

            telemetry.addData("Heading Atual", "%.2f", headingAtual);

            telemetry.addData("Delta Heading", "%.2f", deltaHeading);

            telemetry.addData("Setpoint Torreta", "%.2f", setpointGraus);

            telemetry.addData("Posição Atual", "%.2f", posicaoAtualGraus);

            telemetry.addData("Erro", "%.2f", erro);

            telemetry.addData("Saída", "%.3f", saida);

        }
    }

    private double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    private double angleWrap(double angulo) {
        while (angulo > 180) angulo -= 360;
        while (angulo < -180) angulo += 360;
        return angulo;
    }
}
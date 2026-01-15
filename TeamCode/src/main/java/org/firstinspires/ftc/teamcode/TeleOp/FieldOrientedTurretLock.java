package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Field Oriented + Turret Lock (Stable)", group = "Competition")
public class FieldOrientedTurretLock extends LinearOpMode {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx motorTurret;
    private IMU imu;

    private final double ticks360 = 1.300;
    private final double kP = 0.03;
    private final double maxPower = 0.3;
    private final double giromax = 90.0;
    private final double deadband = 1.0;

    private double headingInicial = 0;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                ));
        imu.initialize(parameters);

        frontLeft = hardwareMap.get(DcMotorEx.class, "motor_esquerda");
        frontRight = hardwareMap.get(DcMotorEx.class, "motor_direita");
        backLeft = hardwareMap.get(DcMotorEx.class, "motor_esquerdatras");
        backRight = hardwareMap.get(DcMotorEx.class, "motor_direitatras");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorTurret = hardwareMap.get(DcMotorEx.class, "motor_turret");
        motorTurret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorTurret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorTurret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Inicializando...");
        telemetry.update();

        waitForStart();

        headingInicial = getHeading();

        while (opModeIsActive()) {
            double headingAtual = getHeading();

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double botHeading = Math.toRadians(headingAtual - headingInicial);

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
            double flPower = (rotY + rotX + rx) / denominator;
            double blPower = (rotY - rotX + rx) / denominator;
            double frPower = (rotY - rotX - rx) / denominator;
            double brPower = (rotY + rotX - rx) / denominator;

            frontLeft.setPower(flPower);
            backLeft.setPower(blPower);
            frontRight.setPower(frPower);
            backRight.setPower(brPower);

            if (gamepad1.b) {
                imu.resetYaw();
                headingInicial = getHeading();
            }

            double deltaHeading = angleWrap(headingAtual - headingInicial);
            double setpointGraus = deltaHeading;

            double posicaoAtualGraus = (motorTurret.getCurrentPosition() / ticks360) * 360.0;

            double erro = angleWrap(setpointGraus - posicaoAtualGraus);

            if (posicaoAtualGraus + erro > giromax) erro -= 180;
            if (posicaoAtualGraus + erro < -giromax) erro += 180;

            if (Math.abs(erro) < deadband) erro = 0;

            double saida = Range.clip(kP * erro, -maxPower, maxPower);
            motorTurret.setPower(saida);

            // ---- TELEMETRIA ----
            telemetry.addData("Heading Atual", "%.2f", headingAtual);
            telemetry.addData("Posição Turret", "%.2f", posicaoAtualGraus);
            telemetry.addData("Erro", "%.2f", erro);
            telemetry.addData("Saída", "%.2f", saida);
            telemetry.update();
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

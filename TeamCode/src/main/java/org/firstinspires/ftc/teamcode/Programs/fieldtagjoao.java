package org.firstinspires.ftc.teamcode.Programs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "fieldtagjao")
public class fieldtagjoao extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;
    private double initialHeading;
    private final double kP = 2.3;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "motor_esquerda");
        frontRight = hardwareMap.get(DcMotor.class, "motor_direita");
        backLeft = hardwareMap.get(DcMotor.class, "motor_esquerdatras");
        backRight = hardwareMap.get(DcMotor.class, "motor_direitatras");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");

        waitForStart();

        initialHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx;

            double currentHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            if (Math.abs(gamepad1.right_stick_x) > 0.1) {
                rx = gamepad1.right_stick_x;
            } else {
                double erro = calculateHeadingError(initialHeading, currentHeading);
                rx = erro * kP;
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            telemetry.addData("Heading Atual", "%.2f", Math.toDegrees(currentHeading));
            telemetry.addData("Heading Inicial", "%.2f", Math.toDegrees(initialHeading));
            telemetry.addData("Modo", (Math.abs(gamepad1.right_stick_x) > 0.1) ? "Manual" : "Corrigindo");
            telemetry.update();
        }
    }

    private double calculateHeadingError(double target, double current) {
        double error = target - current;
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;
        return error;
    }
}
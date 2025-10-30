/*package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.feedback.AngleType;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;

public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();
    private Turret() { }

    private MotorEx motor = new MotorEx("motor_turret");

    private final ControlSystem controlSystem = ControlSystem.builder()
            .angular(AngleType.DEGREES, feedback -> feedback.posPid(0.2, 0, 0))
            .build();


    public Command reto = new RunToPosition(controlSystem, 0).requires(this);
    public Command toMiddle = new RunToPosition(controlSystem, 500).requires(this);
    public Command toHigh = new RunToPosition(controlSystem, 1200).requires(this);

    private IMU imu;
    private double headingInicial = 0;
    private boolean headingOn = false;
    private final double kP = 0.2;
    private final double ticks360 = 1000.0;

    public void initIMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                ));
        imu.initialize(parameters);
    }

    public void headingOn() {
        headingInicial = getHeading();
        headingOn = true;
    }

    public void headingOff() {
        headingOn = false;
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
    private double getSafeTurretTarget(double currentAngle, double targetAngle) {
        currentAngle = AngleUnit.normalizeDegrees(currentAngle);
        targetAngle = AngleUnit.normalizeDegrees(targetAngle);

        double delta = AngleUnit.normalizeDegrees(targetAngle - currentAngle);
        if (Math.abs(delta) > 90) {
            delta += (delta > 0) ? -360 : 360;
        }

        double safeTarget = AngleUnit.normalizeDegrees(currentAngle + delta);

        final double maxesq = -90;
        final double maxdir = 90;
        return Math.max(maxesq, Math.min(maxdir, safeTarget));
    }

    @Override
    public void periodic() {
        if (headingOn && imu != null) {
            double headingAtual = getHeading();
            double deltaHeading = angleWrap(headingAtual - headingInicial);

            double posicaoAtualGraus = (motor.getCurrentPosition() / ticks360) * 360.0;
            double setpointGraus = getSafeTurretTarget(posicaoAtualGraus, deltaHeading);
            double erro = angleWrap(setpointGraus - posicaoAtualGraus);
            double saida = Range.clip(kP * erro, -1, 1);
            motor.setPower(saida);
        } else {
            motor.setPower(controlSystem.calculate(motor.getState()));
        }
    }
}
*/
package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;


@Configurable
public class Shooter implements Subsystem {
    private Localizer localizer;

    public static final Shooter INSTANCE = new Shooter();

    public static double tolerancia = 300;

    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;

    public static double kv = 0;
    public static double ka = 0;
    public static double ks = 0;


    private double robotX = 0;
    private double robotY = 0;

    private double targetX = 144;
    private double targetY = 72;

    private double velTarget = 0;

    private final MotorEx motor1 = new MotorEx("motor_shooter").reversed();
    private final MotorEx motor2 = new MotorEx("motor_shooter2").reversed();
    private final MotorGroup motors = new MotorGroup(motor1, motor2);

    private final ControlSystem controlSystem = ControlSystem.builder()
            .velPid(kp, ki, kd)
            .basicFF(kv, ka, ks)
            .build();

    private Shooter() {}

    private double flywheelSpeed(double dist) {
        return Math.max(
                Math.min(
                        0.0204772 * Math.pow(dist, 2)
                                + 0.643162 * dist
                                + 712.90909,
                        2200
                ),
                0
        );
    }

    public void updateRobotPosition(double x, double y) {
        this.robotX = x;
        this.robotY = y;
    }

    public void setTargetPosition(double x, double y) {
        this.targetX = x;
        this.targetY = y;
    }

    private void updateVelocityFromPosition() {
        double distance = Math.hypot(targetX - robotX, targetY - robotY);
        this.velTarget = flywheelSpeed(distance);
    }

    public boolean isAtTargetVelocity() {
        double currentVelocity = motors.getVelocity();
        return Math.abs(currentVelocity - velTarget) < tolerancia;
    }

    @Override
    public void periodic() {
        updateVelocityFromPosition();

        double power = controlSystem.calculate(
                new KineticState(velTarget, motors.getVelocity())
        );
        motors.setPower(power);
        motors.setPower(ShooterConstants.flywheelSpeed(localizer.getPose().distanceFrom(new Pose(132, 137))));
    }
}

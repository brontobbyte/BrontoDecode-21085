package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@Configurable
public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();
    public static double kp;
    public static double velShoot;

    private final MotorEx motor1 = new MotorEx("motor_shooter").reversed();
    private final MotorEx motor2 = new MotorEx("motor_shooter2").reversed();
    private final MotorGroup motors = new MotorGroup(motor1, motor2);
    private final ControlSystem controlSystem = ControlSystem.builder()
            .velPid(0.001, 0.0, 0.006)
            .basicFF(0.00043, 0.0001, 0.08)
            .build();

    public final Command shoot = new RunToVelocity(controlSystem, -2200, 200).requires(this);
    public final Command parado = new RunToVelocity(controlSystem, 0, new KineticState(0, 0.0)).requires(this);

    private Shooter() { }

    public boolean isAtTargetVelocity() {
        double targetVelocity = -2200;
        double currentVelocity = motors.getVelocity();
        double tolerance = 300;

        return Math.abs(currentVelocity - targetVelocity) < tolerance;
    }

    @Override
    public void periodic() {
        velShoot = motors.getVelocity();
        double power = controlSystem.calculate(motors.getState());
        motors.setPower(power);
    }
}

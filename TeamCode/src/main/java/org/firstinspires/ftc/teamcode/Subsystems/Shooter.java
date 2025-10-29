package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@Config
public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();
    private final MotorEx motor1 = new MotorEx("motor_shooter");
    private final MotorEx motor2 = new MotorEx("motor_shooter2").reversed();

    private final MotorGroup motors = new MotorGroup(motor1, motor2);

    private final ControlSystem controlSystem = ControlSystem.builder()
            .velPid(5.0, 0.0, 0.0)
            .basicFF(0.0003, 0.0001, 0.05)
            .build();

    public final Command shoot = new RunToVelocity(controlSystem, 500).requires(this);
    public final Command shooterparado = new RunToVelocity(controlSystem, 0).requires(this);
    public final Command intake = new RunToVelocity(controlSystem, -200).requires(this);
    public final Command shootando = new SetPower(motors, 1).requires(this);

    private Shooter() { }

    @Override
    public void periodic() {
        motors.setPower(controlSystem.calculate(motors.getState()));
    }
}

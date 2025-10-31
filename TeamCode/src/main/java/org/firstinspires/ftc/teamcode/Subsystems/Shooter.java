package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

@Config
public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();
    private final MotorEx motor1 = new MotorEx("motor_shooter");
    private final MotorEx motor2 = new MotorEx("motor_shooter2");


    private final MotorGroup motors = new MotorGroup(motor1, motor2);
    private final ControlSystem controlSystem = ControlSystem.builder()
            .velPid(0.000001, 0.0, 0.04)
            .basicFF(0.05, 0.0007, 0.0006)
            .build();

    public final Command shoot = new RunToVelocity(controlSystem, 2000, new KineticState(0, 50.0)).requires(this);
    public final Command parado = new RunToVelocity(controlSystem, 0, new KineticState(0, 0.0)).requires(this);

    private Shooter() { }

    @Override
    public void periodic() {
        double power = controlSystem.calculate(motors.getState());
        motors.setPower(power);

    }
}
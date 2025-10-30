package org.firstinspires.ftc.teamcode.Subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class intake implements Subsystem {
    public static final intake INSTANCE = new intake();
    private intake() { }

    private MotorEx motor = new MotorEx("motor_intake");

    private ControlSystem controlSystem = ControlSystem.builder()
            .posPid(0.005, 0, 0)
            .elevatorFF(0)
            .build();

    public Command pega = new SetPower(motor, 1).requires(this);
    public Command stop = new SetPower(motor, 0).requires(this);
    public Command shooting = new RunToPosition(controlSystem, 200).requires(this);

    @Override
    public void periodic() {
        motor.setPower(controlSystem.calculate(motor.getState()));
    }
}
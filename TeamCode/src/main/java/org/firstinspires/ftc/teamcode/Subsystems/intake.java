package org.firstinspires.ftc.teamcode.Subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class intake implements Subsystem {
    public static final intake INSTANCE = new intake();
    private final ControlSystem controlSystem = ControlSystem.builder()
            .velPid(1, 0.0, 0.0)
            .basicFF(0.0, 0.0, 0.0)
            .build();

    private intake() { }
    private MotorEx motor = new MotorEx("motor_intake");

    public Command pega = new SetPower(motor, 1).requires(this);
    public Command shooting = new SetPower(motor, 0.7).requires(this);
    public Command stop = new SetPower(motor, 0).requires(this);

    public Command runContinuously = new LambdaCommand()
            .setStart(() -> motor.setPower(0.7))
            .setStop(interrupted -> motor.setPower(0))
            .setIsDone(() -> false)
            .requires(this)
            .named("Run Intake Continuously");

    @Override
    public void periodic() {
    }
}
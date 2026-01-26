package org.firstinspires.ftc.teamcode.Subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private final ControlSystem controlSystem = ControlSystem.builder()
            .velPid(1, 0.0, 0.0)
            .basicFF(0.0, 0.0, 0.0)
            .build();

    private Intake() { }
    private MotorEx motor = new MotorEx("motor_intake");
    public Command intake = new SetPower(motor, 1).requires(this);
    public Command shooting = new SetPower(motor, 1).requires(this);
    public Command expelir = new SetPower(motor, -1).requires(this);
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
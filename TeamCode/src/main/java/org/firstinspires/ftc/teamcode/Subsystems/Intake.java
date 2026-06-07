package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@Configurable
public class Intake implements Subsystem {

    public static final Intake INSTANCE = new Intake();
    private Intake() {}

    private MotorEx motor = new MotorEx("intake");

    @Override
    public void initialize() {
        //intake.setPower(0.000001);
    }
    public Command stop = new SetPower(motor, 0).requires(this);
    public Command intake = new SetPower(motor, 1).requires(this);
    public Command reversed = new SetPower(motor, -1).requires(this);

    public Command shooting = new SetPower(motor, 1).requires(this);

    @Override
    public void periodic() {
        motor.setPower(1);
    }
}
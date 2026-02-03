package org.firstinspires.ftc.teamcode.Subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    public double power;
    private Intake() { }
    private MotorEx motor = new MotorEx("intake");
    public Command intake = new SetPower(motor, 1).requires(this);
    public Command shooting = new SetPower(motor, 0.8).requires(this);
    public Command expelir = new SetPower(motor, -1).requires(this);
    public Command stop = new SetPower(motor, 0).requires(this);

    @Override
    public void periodic() {
    }
}
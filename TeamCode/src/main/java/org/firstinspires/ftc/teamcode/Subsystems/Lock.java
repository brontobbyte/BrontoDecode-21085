package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.Constants.autoshoot;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;
public class Lock implements Subsystem {
    public static final Lock INSTANCE = new Lock();
    @Override
    public void initialize(){

    }
    private Lock() { }
    private ServoEx servo = new ServoEx("sLock");

    public Command open = new SetPosition(servo, 0.2).requires(this);

    public Command closed = new SetPosition(servo,0).requires(this);
    public double getPos() {
        return servo.getPosition();
    }
    @Override
    public void periodic() {
    }
}

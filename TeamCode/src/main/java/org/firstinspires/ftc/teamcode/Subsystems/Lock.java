package org.firstinspires.ftc.teamcode.Subsystems;

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

    public Command open = new SetPosition(servo, 0.85).requires(this);

    public Command closed = new SetPosition(servo, 1).requires(this);

    @Override
    public void periodic() {
    }
}

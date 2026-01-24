package org.firstinspires.ftc.teamcode.Subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.powerable.SetPower;
public class servoLock implements Subsystem {
    public static final servoLock INSTANCE = new servoLock();
    private servoLock() { }
    private CRServoEx servo = new CRServoEx("sLock");

    public Command puxa = new SetPower(servo, -1.0).requires(this);

    public Command empurra = new SetPower(servo, 1.0).requires(this);

    public Command para = new SetPower(servo, 0).requires(this);

    @Override
    public void periodic() {
    }
}

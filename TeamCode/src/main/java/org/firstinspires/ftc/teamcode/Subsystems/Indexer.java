package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.Constants.autoshoot;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;
public class Indexer implements Subsystem {
    public static final Indexer INSTANCE = new Indexer();
    @Override
    public void initialize(){
    }
    private Indexer() { }
    private ServoEx servo = new ServoEx("sIndexer");

    public Command shooting = new SetPosition(servo, 0.5).requires(this);

    public Command naoshooting = new SetPosition(servo,  0.6).requires(this);
    public double getPos() {
        return servo.getPosition();
    }

    @Override
    public void periodic() {
    }
}

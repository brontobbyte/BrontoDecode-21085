package org.firstinspires.ftc.teamcode.Subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
public class sHood implements Subsystem {
    public static final sHood INSTANCE = new sHood();
    private sHood() { }
    private final ServoEx servoHood = new ServoEx("sHood");
    public final Command alto = new SetPosition(servoHood, 0.85).requires(this);
    public final Command medio = new SetPosition(servoHood, 0.685).requires(this);
    public final Command lock = new SetPosition(servoHood, 0.35).requires(this);

}
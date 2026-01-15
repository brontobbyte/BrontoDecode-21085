package org.firstinspires.ftc.teamcode.Subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
public class Angulador implements Subsystem {
    public static final Angulador INSTANCE = new Angulador();
    private Angulador() { }
    private final ServoEx servo = new ServoEx("servo_angulador");
    private final ServoEx servo2 = new ServoEx("servo_angulador2");
    public final Command alto = new SetPosition(servo, 0.85).requires(this);
    public final Command alto2 = new SetPosition(servo2, 0.15).requires(this);
    public final Command medio = new SetPosition(servo, 0.685).requires(this);
    public final Command medio2 = new SetPosition(servo2, 0.215).requires(this);
    public final Command lock = new SetPosition(servo, 0.35).requires(this);
    public final Command lock2 = new SetPosition(servo2, 0.65).requires(this);

}
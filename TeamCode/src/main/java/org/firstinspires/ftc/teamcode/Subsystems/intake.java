package org.firstinspires.ftc.teamcode.Subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

public class intake implements Subsystem {

    public static final intake INSTANCE = new intake();
    private intake() { }

    private final MotorEx motorIntake = new MotorEx("motor_intake").reversed();

    private final ControlSystem controlSystem = ControlSystem.builder()
            .velPid(1, 0, 0)
            .build();

    public final Command ON = new RunToVelocity(controlSystem, 600).requires(this); // 600

    public final Command OFF = new RunToVelocity(controlSystem, 0).requires(this);

    @Override
    public void periodic() {
    }
}

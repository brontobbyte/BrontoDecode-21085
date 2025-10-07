package org.firstinspires.ftc.teamcode.Subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;

public class TurretServo implements Subsystem {
    public static final TurretServo INSTANCE = new TurretServo();
    private TurretServo() { }
    private MotorEx motorTurret = new MotorEx("motor_turret");


    ControlSystem controlSystem = ControlSystem.builder()
            .posPid(0.0007, 0, 0)
            .build();


    public Command turret = new RunToPosition(controlSystem, 1000).requires(this);

    @Override
    public void periodic() {
        motorTurret.setPower(controlSystem.calculate(motorTurret.getState()));
        // periodic logic (runs every loop)
    }
}

package org.firstinspires.ftc.teamcode.Subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();

    public static double Fkp = 0.01;
    public static double Fki = 0;
    public static double Fkd = 0;
    public static double Fks = 0;
    public static double Fka = 0;
    public static double Fkv = 0;

    private Shooter() { }

    MotorGroup Flywheel = new MotorGroup(
            new MotorEx("f1"),
            new MotorEx("f2")
    );

    private ControlSystem controlSystem = ControlSystem.builder()
            .posPid(Fkp, Fki, Fkd)
            .basicFF(Fkv,Fka,Fks)
            .build();

    public Command off  = new RunToVelocity(controlSystem, 0).requires(this);
    public Command mid  = new RunToVelocity(controlSystem, 7000).requires(this);
    public Command mid2 = new RunToVelocity(controlSystem, 2000).requires(this);
    public Command high = new RunToVelocity(controlSystem, 2500).requires(this);

    @Override
    public void periodic() {
        double power = controlSystem.calculate();
        Flywheel.setPower(power);
    }

    public double getVelocity() {
        return Flywheel.getVelocity();
    }

    public double getPower() {
        return Flywheel.getPower();
    }
}

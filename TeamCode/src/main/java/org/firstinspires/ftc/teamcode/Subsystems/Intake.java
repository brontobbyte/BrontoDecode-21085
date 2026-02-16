package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@Configurable
public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();

    public static double Fkp = 1;
    public static double Fki = 0;
    public static double Fkd = 1;
    public static double Fks = 1;
    public static double Fka = 1;
    public static double Fkv = 1;

    private double targetVelocity = 0;

    private Intake() { }

    private MotorEx motor = new MotorEx("intake");

    private ControlSystem controlSystem = ControlSystem.builder()
            .velPid(Fkp, Fki, Fkd)
            .basicFF(Fkv, Fka, Fks)
            .build();

    public void setVelocity(double velocity) {
        this.targetVelocity = velocity;
        controlSystem.setGoal(new KineticState(0, velocity));
    }

    public Command intake = new SetPower(motor, -1).requires(this);
    public Command shooting = new SetPower(motor, -0.8).requires(this);
    public Command stop = new SetPower(motor, 0).requires(this);

    //public Command stop = new SetPower(motor, 0).requires(this);

    @Override
    public void periodic() {
         // double power = controlSystem.calculate(new KineticState(
        // motor.getCurrentPosition(),
        //motor.getVelocity()));

       // motor.setPower(power);
    }

    public double getVelocity() {
        return motor.getVelocity();
    }

    public double getPower() {
        return motor.getPower();
    }
}
package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.Constants.autoshoot;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();

    public static double Fkp = 0.00003;
    public static double Fki = 0.0000000;
    public static double Fkd = 0.005;
    public static double Fks = 0.1;
    public static double Fka = 6;
    public static double Fkv = 0.00039;

    private double goalDistance = 0;
    public static double goal = 1400;


    private Shooter() { }

    MotorGroup Flywheel = new MotorGroup(
            new MotorEx("f1"),
            new MotorEx("f2")
    );

    @Override
    public void periodic() {
        double targetVelocity = autoshoot.flywheelSpeed(goalDistance);

        ControlSystem controlSystem = ControlSystem.builder()
                .velPid(Fkp, Fki, Fkd)
                .basicFF(Fkv, Fka, Fks)
                .build();

        controlSystem.setGoal(new KineticState(0, goal));

        double power = controlSystem.calculate(new KineticState(
                Flywheel.getCurrentPosition(),
                Flywheel.getVelocity()));


        Flywheel.setPower(power);
    }

    public void setGoalDistance(double dist) {
        this.goalDistance = dist;
    }
    public double getVelocity() {
        return Flywheel.getVelocity();
    }
    public double getPower() {
        return Flywheel.getPower();
    }
}
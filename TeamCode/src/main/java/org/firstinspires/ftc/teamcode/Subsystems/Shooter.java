package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import org.firstinspires.ftc.teamcode.Constants.autoshoot;

@Configurable
public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();

    public static double Fkp =0.005;
    public static double Fki = 0.000000001;
    public static double Fkd = 0;
    public static double Fks = 0.17;
    public static double Fka = 6;
    public static double Fkv = 0.00028;

    private final MotorGroup Flywheel = new MotorGroup(
            new MotorEx("f1"),
            new MotorEx("f2")
    );

    @Override
    public void initialize() {
        Flywheel.setPower(0.000001);
    }

    @Override
    public void periodic() { }

    public void setGoalDistance(double goalDistance) {
        double speed = autoshoot.flywheelSpeed(goalDistance);
        setVelocity(speed);
    }

    public void setVelocity(double launchSpeed) {
        ControlSystem controlSystem = ControlSystem.builder()
                .velPid(Fkp, Fki, Fkd)
                .basicFF(Fkv, Fka, Fks)
                .build();

        controlSystem.setGoal(new KineticState(0, launchSpeed));

        double power = controlSystem.calculate(new KineticState(
                Flywheel.getCurrentPosition(),
                Flywheel.getVelocity()));

        Flywheel.setPower(power);
    }

    public double getVelocity() {
        return Flywheel.getVelocity();
    }
    public double getGoal(double goalDistance) {
        return autoshoot.flywheelSpeed(goalDistance);
    }


    public double getPower() {
        return Flywheel.getPower();
    }
}
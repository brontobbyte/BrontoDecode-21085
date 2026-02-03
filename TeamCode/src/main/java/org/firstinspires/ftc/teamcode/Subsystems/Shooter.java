package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();

    public static double Fkp = 0.0001;
    public static double Fki = 0.000000000001;
    public static double Fkd = 0.00001;
    public static double Fks = 0.3;
    public static double Fka = 1;
    public static double Fkv = 0.0003789;

    private double goalDistance = 0;

    private Shooter() { }

    MotorGroup Flywheel = new MotorGroup(
            new MotorEx("f1"),
            new MotorEx("f2")
    );

    private ControlSystem controlSystem = ControlSystem.builder()
            .velPid(Fkp, Fki, Fkd)
            .basicFF(Fkv, Fka, Fks)
            .build();

    public Command off  = new RunToVelocity(controlSystem, 0).requires(this);
    public Command mid  = new RunToVelocity(controlSystem, 7000).requires(this);
    public Command mid2 = new RunToVelocity(controlSystem, 2000).requires(this);
    public Command high = new RunToVelocity(controlSystem, 2500).requires(this);

    @Override
    public void periodic() {
        double targetVelocity = ShooterConstants.flywheelSpeed(goalDistance);

        telemetry.addData("Shooter Goal Distance", goalDistance);
        telemetry.addData("Shooter Target Velocity", targetVelocity);

        controlSystem.setGoal(new KineticState(targetVelocity));

        double power = controlSystem.calculate(new KineticState(
                Flywheel.getCurrentPosition(),
                Flywheel.getVelocity()));

        telemetry.addData("Shooter Calculated Power", power);
        telemetry.addData("Shooter Current Velocity", Flywheel.getVelocity());

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

    public void shoot() {
        telemetry.addData("Shooting", ShooterConstants.launchTime(goalDistance));
    }
}
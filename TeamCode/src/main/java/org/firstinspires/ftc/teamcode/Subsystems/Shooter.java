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
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

@Configurable
public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();

    public static double Fkp = 0.00001;
    public static double Fki = 0;
    public static double Fkd = 0.001;
    public static double Fks = 0.179;
    public static double Fka = 2;
    public static double Fkv = 0.0007;
    public static double vel = 1700;

    private double goalDistance = 0;

    private Shooter() { }

    MotorGroup Flywheel = new MotorGroup(
            new MotorEx("f1"),
            new MotorEx("f2")
    );

    @Override
    public void periodic() {
        double targetVelocity = ShooterConstants.flywheelSpeed(goalDistance);
        ControlSystem controlSystem = ControlSystem.builder()
                .velPid(Fkp, Fki, Fkd)
                .basicFF(Fkv, Fka, Fks)
                .build();

        controlSystem.setGoal(new KineticState(0, (targetVelocity+ShooterConstants.getFlywheelOffset())));

        double power = controlSystem.calculate(new KineticState(
                Flywheel.getCurrentPosition(),
                Flywheel.getVelocity()));

        Flywheel.setPower(power);
    }

    public double getVelocity() {
        return Flywheel.getVelocity();
    }

    public double getPower() {
        return Flywheel.getPower();
    }

    public void shoot() {
    }
}
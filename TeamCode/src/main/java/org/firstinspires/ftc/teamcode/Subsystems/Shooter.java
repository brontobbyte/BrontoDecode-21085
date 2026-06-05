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

    public static double Fkp  = 0.0;
    public static double Fki  = 0.00000000001;
    public static double Fkd  = 0.0;
    public static double Fks  = 0.21;
    public static double Fka  = 0.0;
    public static double Fkv  = 0.00034;
    public static double goal = 1500;

    private final MotorGroup flywheel = new MotorGroup(
            new MotorEx("f1").reversed(),
            new MotorEx("f2").reversed()
    );

    private ControlSystem controlSystem;
    private double targetVelocity = 0.0;
    private boolean running = false;

    @Override
    public void initialize() {
        rebuildControlSystem();
        flywheel.setPower(0.0);
    }

    @Override
    public void periodic() {
        if (!running || controlSystem == null) {
            flywheel.setPower(0.0);
            return;
        }

        double power = controlSystem.calculate(
                new KineticState(
                        flywheel.getCurrentPosition(),
                        flywheel.getVelocity()
                )
        );

        flywheel.setPower(power);
    }

    public void rebuildControlSystem() {
        controlSystem = ControlSystem.builder()
                .velPid(Fkp, Fki, Fkd)
                .basicFF(Fkv, Fka, Fks)
                .build();

        controlSystem.setGoal(new KineticState(0.0, targetVelocity));
    }

    public void setVelocity(double velocityTicksPerSec) {
        targetVelocity = velocityTicksPerSec;
        running = true;

        if (controlSystem != null) {
            controlSystem.setGoal(new KineticState(0.0, velocityTicksPerSec));
        }
    }

    public void setGoalDistance(double goalDistance) {
        setVelocity(autoshoot.flywheelSpeed(goalDistance));
    }

    public void stop() {
        running = false;
        targetVelocity = 0.0;

        if (controlSystem != null) {
            controlSystem.setGoal(new KineticState(0.0, 0.0));
        }

        flywheel.setPower(0.0);
    }

    @Deprecated
    public void setSpeed(double speed) {
        if (speed == 0.0) stop();
        else setVelocity(speed);
    }

    public double getVelocity() {
        return flywheel.getVelocity();
    }

    public double getPower() {
        return flywheel.getPower();
    }

    public double getTarget() {
        return targetVelocity;
    }

    public boolean isRunning() {
        return running;
    }

    public double getVelocityError() {
        return targetVelocity - flywheel.getVelocity();
    }

    public double getGoalForDistance(double goalDistance) {
        return autoshoot.flywheelSpeed(goalDistance);
    }
}
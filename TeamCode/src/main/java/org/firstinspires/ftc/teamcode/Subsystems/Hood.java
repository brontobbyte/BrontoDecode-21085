package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

import org.firstinspires.ftc.teamcode.Constants.autoshoot;

@Configurable
public class Hood implements Subsystem {

    public static final Hood INSTANCE = new Hood();
    private Hood() {}

    private final ServoEx servoHood = new ServoEx("sHood");

    public static double pos = 0.7;

    private double pendingPosition = 0.75;
    private boolean positionDirty  = false;

    public Command set = new SetPosition(servoHood, 0.61).requires(this);

    @Override
    public void initialize() {
        servoHood.setPosition(0.75);
        pendingPosition = 0.75;
        positionDirty   = false;
    }

    @Override
    public void periodic() {
        if (positionDirty) {
            servoHood.setPosition(pendingPosition);
            positionDirty = false;
        }
    }

    public void setGoalDistance(double goalDistance) {
        setHoodPosition(autoshoot.hoodAngle(goalDistance));
    }

    public void setHoodPosition(double position) {
        pos = Math.min(Math.max(position, 0.28), 0.98);
        pendingPosition = pos;
        positionDirty   = true;
    }

    public void setHoodPos(double position) {
        setHoodPosition(position);
    }

    public double getHoodPosition() {
        return pos;
    }
}
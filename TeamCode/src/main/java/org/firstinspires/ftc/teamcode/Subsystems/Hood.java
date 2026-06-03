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
    private Hood() { }
    private final ServoEx servoHood = new ServoEx("sHood");
    public static double pos = 0.8;
    public void initialize() {
        servoHood.setPosition(0.75);
    }
    @Override
    public void periodic() {
    }
    public Command set = new SetPosition(servoHood, 0.61).requires(this);

    public void setHoodPos(double position){
        servoHood.setPosition(position);
    }
    public void setGoalDistance(double goalDistance) {
        double angle = autoshoot.hoodAngle(goalDistance);
        setHoodPosition(angle);

    }

    public void setHoodPosition(double launchAngle) {
        pos = Math.min(Math.max(launchAngle, 0.28), 0.98);
        servoHood.setPosition(pos);
    }
    public double getHoodPosition() {
        return pos;
    }
}
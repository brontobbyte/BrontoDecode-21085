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

    private double goalDistance = 0;
    public static double pos = 0.1;

    private Hood() { }

    private final ServoEx servoHood = new ServoEx("sHood");

    public final Command alto = new SetPosition(servoHood, 0.92).requires(this);
    public final Command medio = new SetPosition(servoHood, 0.45).requires(this);
    public final Command baixo = new SetPosition(servoHood, 0.1).requires(this);

    @Override
    public void periodic() {
        if (goalDistance > 0) {
            double hoodPosition = autoshoot.hoodAngle(goalDistance);
            servoHood.setPosition(hoodPosition);
        }
    }

    public void setGoalDistance(double dist) {
        this.goalDistance = dist;
    }
}
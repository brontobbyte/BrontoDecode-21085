package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;

import org.firstinspires.ftc.teamcode.Constants.autoshoot;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

@Configurable
public class Hood implements Subsystem {
    public static final Hood INSTANCE = new Hood();

    private double goalDistance = 0;
    private double pos = 0.1;

    private Hood() { }

    private final ServoEx servoHood = new ServoEx("sHood");

    public final Command alto = new SetPosition(servoHood, 0.6).requires(this);
    public final Command medio = new SetPosition(servoHood, 0.45).requires(this);
    public final Command baixo = new SetPosition(servoHood, 0.25).requires(this);


    @Override
    public void periodic() {

        if (goalDistance > 0) {
            //double hoodAngle = autoshoot.hoodAngle(goalDistance);
            servoHood.setPosition(pos);
        } else {
        }
    }

    public void setGoalDistance(double dist) {
        this.goalDistance = dist;
    }

}
package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;


@Configurable
public class Hood implements Subsystem {
    public static final Hood INSTANCE = new Hood();

    private Hood() { }
    public static double pos = 0.3;
    private double goalDistance = 0;
    public void setGoalDistance(double dist) {
        this.goalDistance = dist;
    }
    private final ServoEx servoHood = new ServoEx("sHood");

    public final Command alto = new SetPosition(servoHood, 0.85).requires(this);
    public final Command medio = new SetPosition(servoHood, 0.685).requires(this);
    public final Command lock = new SetPosition(servoHood, 0.35).requires(this);

    @Override
    public void periodic() {
        double targetAngle = ShooterConstants.hoodAngle(goalDistance);
        servoHood.setPosition(targetAngle);
    }
}
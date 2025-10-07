package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import kotlin.io.TextStreamsKt;

@Config
public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();
    private Shooter() { }
    private MotorEx motorShooter = new MotorEx("motor_shooter")
            .reversed();
    private MotorEx motorShooter2 = new MotorEx("motor_shooter2");
    private MotorGroup motoresShooter = new MotorGroup(motorShooter, motorShooter2);
    public static double p = 1;
    public static double i = 0;
    public static double d = 0;
    public static double vel, vel1;
    public static double pos;
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dash = FtcDashboard.getInstance();
    Telemetry dashTelemetry = dash.getTelemetry();
    ControlSystem controlSystem = ControlSystem.builder()
            .velPid(0.000214, 0, 0)
            .build();

    public Command ON = new RunToVelocity(controlSystem, 10000).requires(this); //10000 para 1
    public Command OFF = new RunToVelocity(controlSystem, 0).requires(this);
    public Command Stop() {
        return new RunToVelocity(controlSystem, 0).requires(this);
    }
    public Command ONClass() {
        return new RunToVelocity(controlSystem, 600).requires(this);
    }



    @Override
    public void periodic() {
        pos = controlSystem.calculate(new KineticState(motorShooter.getCurrentPosition()));
        vel = controlSystem.calculate(new KineticState(motorShooter.getVelocity()));
        vel1 = motorShooter.getVelocity();
        KineticState vel2 = new KineticState(motorShooter.getVelocity());
        motoresShooter.setPower(controlSystem.calculate(motoresShooter.getState()));
        // periodic logic (runs every loop)
    }


}
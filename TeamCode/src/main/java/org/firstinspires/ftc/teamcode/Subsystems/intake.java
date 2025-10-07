package org.firstinspires.ftc.teamcode.Subsystems;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

@Config
public class intake implements Subsystem {
    public static final intake INSTANCE = new intake();
    private intake() { }
    private final MotorEx motorintake= new MotorEx("motor_intake")
            .reversed();
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

    public Command coletar = new RunToVelocity(controlSystem, 10000).requires(this); //10000 para 1
    public Command para = new RunToVelocity(controlSystem, 0).requires(this);
    public Command Stop() {
        return new RunToVelocity(controlSystem, 0).requires(this);
    }
    public Command ONClass() {
        return new RunToVelocity(controlSystem, 600).requires(this);
    }


    @Override
    public void periodic() {

    }


}
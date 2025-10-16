package org.firstinspires.ftc.teamcode.Subsystems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();
    private Shooter() { }
    private MotorEx motorShooter = new MotorEx("motor_shooter")
            .reversed();
    private MotorEx motorShooter2 = new MotorEx("motor_shooter2");
    private MotorGroup motoresShooter = new MotorGroup(motorShooter, motorShooter2);

    public static double p = 0.011;
    public static double i = 0;
    public static double d = 0.005;
    public static PIDCoefficients coefficients = new PIDCoefficients(p, i, d);

    public static double vel, vel1;
    public static double pos;
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dash = FtcDashboard.getInstance();
    Telemetry dashTelemetry = dash.getTelemetry();
    ControlSystem controlSystem = ControlSystem.builder()
            .velPid(coefficients)
            //.basicFF(0.0, 0.0, 0.0)
            .build();

    public Command shoot = new RunToVelocity(controlSystem, 1600).requires(this); //10000 para 1
    public Command OFF1 = new RunToVelocity(controlSystem, 0).requires(this);
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
package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import static org.firstinspires.ftc.teamcode.Programs.TeleopTeste.yaw;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
public class TurretSeguidor implements Subsystem {
    public static final TurretSeguidor INSTANCE = new TurretSeguidor();

    private TurretSeguidor() {
    }
    double posSeguidorGraus;

    private MotorEx motorTurret = new MotorEx("motor_turret")
            .reversed();
    public static double p = 1;
    public static double i = 0;
    public static double d = 0;
    public static double vel, vel1;
    public static double posTurret;
    HardwareMap hardwareMap;
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dash = FtcDashboard.getInstance();
    Telemetry dashTelemetry = dash.getTelemetry();
    ControlSystem controlSystem = ControlSystem.builder()
            .posPid(0.008, 0, 0)
            .build();

    public Command CentoEOitenta = new RunToPosition(controlSystem, 615).requires(this); //10000 para 1
    public Command Noventa = new RunToPosition(controlSystem, 307.5).requires(this); //10000 para 1
    public Command Stop() {
        return new RunToVelocity(controlSystem, 0).requires(this);
    }
    public Command ONClass() {
        return new RunToVelocity(controlSystem, 600).requires(this);
    }
    public Command seguidor = new RunToPosition(controlSystem, posSeguidorGraus).requires(this); //10000 para 1

    @Override
public void initialize(){

}

    @Override
    public void periodic() {
        new RunToPosition(controlSystem, posSeguidorGraus).requires(this);

        posTurret = motorTurret.getCurrentPosition();
        motorTurret.setPower(controlSystem.calculate(motorTurret.getState()));
        // periodic logic (runs every loop)
    }


}
package org.firstinspires.ftc.teamcode.Programs;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Subsystems.Shooter.pos;
import static org.firstinspires.ftc.teamcode.Subsystems.Shooter.vel;
import static org.firstinspires.ftc.teamcode.Subsystems.Shooter.vel1;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.posTurret;
import static dev.nextftc.bindings.Bindings.button;

import android.renderscript.ScriptGroup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSeguidor;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Bindings;
import dev.nextftc.bindings.Bindings.*;
import dev.nextftc.bindings.Button;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.delegates.Velocity;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "TeleOpTeste")
@Config
public class TeleopTeste extends NextFTCOpMode {

    {
        addComponents(
                new SubsystemComponent(Shooter.INSTANCE, Turret.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    public static double yaw;
    public static double p = 0.0006;
    public static double i = 0;
    public static double d = 0.00001;
    double posSeguidorGraus;
    private IMU imu;
    private MotorEx motorTurret = new MotorEx("motor_turret")
            .reversed()
            .zeroed();

    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dash = FtcDashboard.getInstance();
    Telemetry dashTelemetry = dash.getTelemetry();
    ControlSystem controlSystem = ControlSystem.builder()
            .posPid(p, i, d)
            .build();
    MotorEx motorShooter = new MotorEx("motor_shooter");


    @Override public void onInit(){
        imu = hardwareMap.get(IMU.class, "imu");
        motorTurret.zeroed();
    }

    @Override public void onStartButtonPressed(){
        Gamepads.gamepad1().a()
                .whenTrue(Shooter.INSTANCE.ON)
                .whenTrue(() -> telemetry.update())
                .whenFalse(() -> telemetry.update())
                .whenFalse(Shooter.INSTANCE.OFF)
                .whenFalse(() -> telemetry.addData("botao", "nada"));
        Gamepads.gamepad1().dpadUp()
                .whenTrue(Turret.INSTANCE.Zero);
        Gamepads.gamepad1().dpadDown()
                .whenTrue(Turret.INSTANCE.CentoEOitenta);
        Gamepads.gamepad1().dpadRight()
                .whenTrue(Turret.INSTANCE.Noventa);
    }
    @Override public void onUpdate(){
        yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double posSeguidor = yaw;
        posSeguidorGraus = ((-posSeguidor*615)/180);
        //motorTurret.setPower(posSeguidorGraus / 90);
        /*Gamepads.gamepad1().b()
                .whenBecomesTrue(TurretSeguidor.INSTANCE.seguidor);
        Gamepads.gamepad1().y()
                .whenTrue(()-> imu.resetYaw());
        motorTurret.setPower(controlSystem.calculate(motorTurret.getState()));*/
        packet.put("Teste", vel);
        packet.put("vel1", vel1);
        packet.put("pos", pos);
        packet.put("posSeguidor", posSeguidorGraus);
        packet.put("posSeguidorMotor", posSeguidorGraus/90);
        packet.put("posAtual", motorTurret.getCurrentPosition());
        packet.put("posTurret", posTurret);
        dash.sendTelemetryPacket(packet);
        dashTelemetry.addData("teste2", vel);
        dashTelemetry.update();
        telemetry.update();
    }

    @Override public void onStop(){
        Shooter.INSTANCE.Stop().schedule();
        BindingManager.reset();
    }
}

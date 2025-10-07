package org.firstinspires.ftc.teamcode.Programs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.indexer;
import org.firstinspires.ftc.teamcode.Subsystems.intake;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "Main TeleOp Completo")
@Config
public class MainTeleOp extends NextFTCOpMode {

    {
        addComponents(
                new SubsystemComponent(Shooter.INSTANCE, Turret.INSTANCE, intake.INSTANCE, indexer.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    public static double yaw;
    private IMU imu;

    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dash = FtcDashboard.getInstance();

    @Override
    public void onInit(){
        imu = hardwareMap.get(IMU.class, "imu");
    }

    @Override
    public void onStartButtonPressed(){
        Gamepads.gamepad1().leftBumper()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(intake.INSTANCE.coletar)
                .whenBecomesFalse(intake.INSTANCE.para);

        // Shooter - Right Bumper (toggle)
        Gamepads.gamepad1().rightBumper()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Shooter.INSTANCE.ON)
                .whenBecomesFalse(Shooter.INSTANCE.OFF);

        Gamepads.gamepad1().a()
                .whenTrue(indexer.INSTANCE.empurra)
                .whenFalse(indexer.INSTANCE.puxa);

        // Turret - Posições com D-Pad
        Gamepads.gamepad1().dpadUp()
                .whenTrue(Turret.INSTANCE.Zero);
        Gamepads.gamepad1().dpadDown()
                .whenTrue(Turret.INSTANCE.CentoEOitenta);
        Gamepads.gamepad1().dpadRight()
                .whenTrue(Turret.INSTANCE.Noventa);
        Gamepads.gamepad1().dpadLeft()
                .whenTrue(Turret.INSTANCE.Stop());
    }

    @Override
    public void onUpdate(){
        yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        packet.put("Yaw IMU", yaw);

        dash.sendTelemetryPacket(packet);
        telemetry.update();
    }

    @Override
    public void onStop(){
        Shooter.INSTANCE.OFF.schedule();
        intake.INSTANCE.para.schedule();
        indexer.INSTANCE.empurra.schedule();
    }
}
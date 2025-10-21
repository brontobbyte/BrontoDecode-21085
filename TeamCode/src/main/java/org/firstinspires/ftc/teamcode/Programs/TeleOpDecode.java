package org.firstinspires.ftc.teamcode.Programs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
//import org.firstinspires.ftc.teamcode.Subsystems.Intake;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@TeleOp(name = "TeleOpTeste")
@Config
public class TeleOpDecode extends NextFTCOpMode {

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
    ControlSystem controlSystem = ControlSystem.builder()
            .posPid(p, i, d)
            .build();
    MotorEx motorShooter = new MotorEx("motor_shooter");
    CRServoEx ServoIndexer = new CRServoEx("servo_indexer");


    @Override public void onInit(){
        imu = hardwareMap.get(IMU.class, "imu");
        motorTurret.zeroed();
    }

    @Override public void onStartButtonPressed(){
        Gamepads.gamepad1().rightBumper()
                .whenTrue(new SequentialGroup(
                        new SetPower(ServoIndexer, 1),
                        new Delay(500),
                        new SetPower(ServoIndexer, 0),
                        //Intake.INSTANCE.coletar,
                        new Delay(500)
                        //Intake.INSTANCE.para
                        ));
    }
    @Override public void onUpdate(){
        BindingManager.update();
        telemetry.update();
    }

    @Override public void onStop(){
        Shooter.INSTANCE.Stop().schedule();
        BindingManager.reset();
    }
}

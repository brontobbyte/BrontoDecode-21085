package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.SubsystemComponent;

import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.sHood;

@TeleOp(name = "teleop teste")
public class teleteste extends NextFTCOpMode {

    @Override
    public void onInit() {

        addComponents(
                new SubsystemComponent(Turret.INSTANCE,Shooter.INSTANCE,sHood.INSTANCE)
        );
        Turret.INSTANCE.init(hardwareMap);
    }

    @Override
    public void onStartButtonPressed() {

    }

    @Override
    public void onUpdate() {

    }

    @Override
    public void onStop() {
        Turret.INSTANCE.getMotor().setPower(0);
    }
}

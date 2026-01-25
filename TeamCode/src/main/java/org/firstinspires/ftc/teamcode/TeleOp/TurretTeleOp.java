package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.SubsystemComponent;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@TeleOp(name = "Turret Only TeleOp")
public class TurretTeleOp extends NextFTCOpMode {

    @Override
    public void onInit() {
        addComponents(new SubsystemComponent(Turret.INSTANCE));

        Turret.INSTANCE.init(hardwareMap);
    }

    @Override
    public void onStartButtonPressed() {
    }

    @Override
    public void onUpdate() { }

    @Override
    public void onStop() {
        Turret.INSTANCE.getMotor().setPower(0);
    }
}

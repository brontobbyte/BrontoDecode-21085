package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
@Configurable
@TeleOp(name = "NextFTC TeleOp Program Java")
public class TeleOpProgram extends NextFTCOpMode {

    @Override
    public void onInit() {
        addComponents(
                new SubsystemComponent(Shooter.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onStartButtonPressed() {
        Gamepads.gamepad2().x().whenBecomesTrue(Shooter.INSTANCE.off);
        Gamepads.gamepad2().y().whenBecomesTrue(Shooter.INSTANCE.mid);
        Gamepads.gamepad2().b().whenBecomesTrue(Shooter.INSTANCE.high);
        Gamepads.gamepad2().a().whenBecomesTrue(Shooter.INSTANCE.mid2);
    }

    @Override
    public void onUpdate() {
        Shooter.INSTANCE.periodic();

        telemetry.addLine("flywheel");
        telemetry.addData("velocity", Shooter.INSTANCE.getVelocity());
        telemetry.addData("power", Shooter.INSTANCE.getPower());
        telemetry.update();
    }

    @Override
    public void onStop() {
        Shooter.INSTANCE.off.run();


    }
}

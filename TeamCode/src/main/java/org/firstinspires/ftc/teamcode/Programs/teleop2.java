package org.firstinspires.ftc.teamcode.Programs;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.indexer;
import org.firstinspires.ftc.teamcode.Subsystems.intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.extensions.pedro.PedroComponent;

@TeleOp(name = "teleop")
public class teleop2 extends NextFTCOpMode {

    public teleop2() {
        addComponents(
                new SubsystemComponent(Shooter.INSTANCE,intake.INSTANCE, indexer.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    @Override
    public void onStartButtonPressed() {

        Gamepads.gamepad2().rightBumper()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Shooter.INSTANCE.shoot)
                .whenBecomesTrue(indexer.INSTANCE.puxa)
                .whenBecomesFalse(indexer.INSTANCE.STOP)
                .whenBecomesFalse(Shooter.INSTANCE.OFF1);

        Gamepads.gamepad2().leftBumper()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(intake.INSTANCE.ON)
                .whenBecomesFalse(intake.INSTANCE.OFF);

        DriverControlledCommand driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        );
        driverControlled.schedule();

        follower().breakFollowing();
    }
}

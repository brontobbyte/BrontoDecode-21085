package org.firstinspires.ftc.teamcode.Programs;

import static dev.nextftc.bindings.Bindings.button;
import static dev.nextftc.bindings.Bindings.range;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Angulador;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.indexer;
import org.firstinspires.ftc.teamcode.Subsystems.intake;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;

@Configurable
@TeleOp(name = "NextFTC TeleOp Program Java")
public class TeleOpProgram extends NextFTCOpMode {

    public TeleOpProgram() {
        addComponents(
                new SubsystemComponent(indexer.INSTANCE, intake.INSTANCE, Shooter.INSTANCE, Turret.INSTANCE, Angulador.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }
    private final IMUEx imu = new IMUEx("imu", Direction.LEFT, Direction.UP).zeroed();

    @Override
    public void onInit() {
        Turret.INSTANCE.init(hardwareMap);
        Turret.INSTANCE.enableLimelightTracking();
    }
    @Override
    public void onStartButtonPressed() {
        DriverControlledCommand driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate(),
                false
        );
        driverControlled.schedule();

        button(() -> gamepad1.b)
                .whenBecomesTrue(() -> imu.getImu().resetYaw());

        button(() -> gamepad2.left_bumper)
                .whenTrue(() -> intake.INSTANCE.pega.schedule())
                .whenFalse(() -> intake.INSTANCE.stop.schedule());

        button(() -> gamepad2.right_bumper)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() ->
                        new ParallelGroup(
                                Shooter.INSTANCE.shoot,
                                Angulador.INSTANCE.medio,
                                Angulador.INSTANCE.medio2
                        ).schedule()
                )
                .whenBecomesFalse(() ->
                        new ParallelGroup(
                                Shooter.INSTANCE.parado,
                                Angulador.INSTANCE.lock,
                                Angulador.INSTANCE.lock2,
                                indexer.INSTANCE.para
                        ).schedule()
                );

        button(() -> gamepad2.a)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() ->
                        new ParallelGroup(
                                indexer.INSTANCE.puxa
                        ).schedule()
                );
    }
    @Override
    public void onUpdate() {
        super.onUpdate();
        Turret.INSTANCE.periodic();
    }
}

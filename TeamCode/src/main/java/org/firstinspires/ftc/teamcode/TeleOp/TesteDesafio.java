package org.firstinspires.ftc.teamcode.TeleOp;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Angulador;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.indexer;
import org.firstinspires.ftc.teamcode.Subsystems.intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
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


@TeleOp( name = "Prog Desafio" )
public class TesteDesafio extends NextFTCOpMode {
    public TesteDesafio() {
        addComponents(
                new SubsystemComponent(Shooter.INSTANCE, indexer.INSTANCE, intake.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }
    private IMUEx imu = new IMUEx("imu", Direction.LEFT, Direction.UP).zeroed();

    @Override
    public void onStartButtonPressed() {
        DriverControlledCommand driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate(),
                false
        );
        driverControlled.schedule();
        button(() -> gamepad2.b)
                .whenBecomesTrue(() -> imu.getImu().resetYaw());

        button(() -> gamepad2.left_bumper)
                .whenTrue(() -> intake.INSTANCE.pega.schedule())
                .whenFalse(() -> intake.INSTANCE.stop.schedule());

        button(() -> gamepad2.a)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() ->
                        indexer.INSTANCE.puxa.schedule()
                );
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
                        new ParallelRaceGroup(
                                Shooter.INSTANCE.parado,
                                Angulador.INSTANCE.lock,
                                Angulador.INSTANCE.lock2,
                                indexer.INSTANCE.para
                        ).schedule()
                );
    }
}
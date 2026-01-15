package org.firstinspires.ftc.teamcode.TeleOp;

import static dev.nextftc.bindings.Bindings.button;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Angulador;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.indexer;
import org.firstinspires.ftc.teamcode.Subsystems.intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
@TeleOp(name = "ProgramTesteOp")
public class ProgTeleOpComField extends NextFTCOpMode {


    public ProgTeleOpComField() {
        addComponents(
                new SubsystemComponent(indexer.INSTANCE, intake.INSTANCE, Shooter.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    private final IMUEx imu = new IMUEx("imu", Direction.LEFT, Direction.UP).zeroed();
    private final MotorEx frontLeftMotor = new MotorEx("motor_esquerda");
    private final MotorEx frontRightMotor = new MotorEx("motor_direita");
    private final MotorEx backLeftMotor = new MotorEx("motor_esquerdatras");
    private final MotorEx backRightMotor = new MotorEx("motor_direitatras");
    private final MotorEx turret = new MotorEx("motor_turret");


    @Override
    public void onInit() {

    }
    @Override
    public void onStartButtonPressed() {

        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX(),
                new FieldCentric(imu)

        );
        driverControlled.schedule();


        button(() -> gamepad1.b)
                .whenBecomesTrue(() -> imu.getImu().resetYaw());

        button(() -> gamepad2.left_bumper)
                .whenTrue(() -> intake.INSTANCE.pega.schedule())
                .whenFalse(() -> intake.INSTANCE.stop.schedule());

        button(() -> gamepad2.a)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> indexer.INSTANCE.puxa.schedule()
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
                        new ParallelGroup(
                                Shooter.INSTANCE.parado,
                                Angulador.INSTANCE.lock,
                                Angulador.INSTANCE.lock2,
                                indexer.INSTANCE.para
                        ).schedule()
                );
    }

}
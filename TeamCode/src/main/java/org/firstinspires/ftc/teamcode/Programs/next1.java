package org.firstinspires.ftc.teamcode.Programs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.indexer;
import org.firstinspires.ftc.teamcode.Subsystems.intake;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;
import static dev.nextftc.bindings.Bindings.*;

@TeleOp(name = "NextFTC TeleOp Program Java")
public class next1 extends NextFTCOpMode {
    public next1() {
        addComponents(
                new SubsystemComponent(Shooter.INSTANCE, intake.INSTANCE, indexer.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    // change the names and directions to suit your robot
    private final MotorEx frontLeftMotor = new MotorEx("motor_esquerda").reversed();
    private final MotorEx frontRightMotor = new MotorEx("motor_direita");
    private final MotorEx backLeftMotor = new MotorEx("motor_esquerdatras").reversed();
    private final MotorEx backRightMotor = new MotorEx("motor_direitatras");

    @Override
    public void onStartButtonPressed() {
        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        );
        driverControlled.schedule();

        Gamepads.gamepad2().a()
                .whenBecomesTrue(indexer.INSTANCE.puxa)
                .whenBecomesTrue(intake.INSTANCE.pega)
                .whenBecomesFalse(intake.INSTANCE.stop)
                .whenBecomesFalse(indexer.INSTANCE.para);

        Gamepads.gamepad2().leftBumper()
                .whenBecomesTrue(intake.INSTANCE.pega)
                .whenBecomesTrue(Shooter.INSTANCE.intake)
                .whenBecomesTrue(indexer.INSTANCE.empurra)
                .whenBecomesFalse(Shooter.INSTANCE.parado)
                .whenBecomesFalse(indexer.INSTANCE.para)
                .whenBecomesFalse(intake.INSTANCE.stop);

        Gamepads.gamepad2().rightBumper()
                .whenBecomesTrue(Shooter.INSTANCE.shoot)
                .whenBecomesFalse(Shooter.INSTANCE.parado);

    }
}
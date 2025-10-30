package org.firstinspires.ftc.teamcode.Programs;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.indexer;
import org.firstinspires.ftc.teamcode.Subsystems.intake;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "lakfn")
public class testeteleop extends NextFTCOpMode {

    public testeteleop() {
        addComponents(
                new SubsystemComponent(Shooter.INSTANCE, intake.INSTANCE, indexer.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private final MotorEx frontLeftMotor = new MotorEx("motor_esquerda").brakeMode().reversed();
    private final MotorEx frontRightMotor = new MotorEx("motor_direita").brakeMode();
    private final MotorEx backLeftMotor = new MotorEx("motor_esquerdatras").brakeMode().reversed();
    private final MotorEx backRightMotor = new MotorEx("motor_direitatras").brakeMode();
    private final IMUEx imu = new IMUEx("imu", Direction.LEFT, Direction.UP).zeroed();

    @Override
    public void onStartButtonPressed() {

        new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX(),
                new FieldCentric(imu)

        ).schedule();

        button(() -> gamepad2.right_bumper)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> Shooter.INSTANCE.shootando.schedule())
                .whenBecomesFalse(() -> Shooter.INSTANCE.shooterparado.schedule());

        button(() -> gamepad2.left_bumper)
                .whenTrue(() -> intake.INSTANCE.pega.and(Shooter.INSTANCE.intake).schedule())
                .whenFalse(() -> Shooter.INSTANCE.shooterparado.and(intake.INSTANCE.stop));

        button(() -> gamepad1.b)
                .whenBecomesTrue(() -> imu.getImu().resetYaw());
    }
}

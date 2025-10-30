package org.firstinspires.ftc.teamcode.Programs;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.indexer;
import org.firstinspires.ftc.teamcode.Subsystems.intake;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.driving.FieldCentric;
@TeleOp(name = "NextFTC TeleOp Heading Turret")
public class next1 extends NextFTCOpMode {

    private MotorEx frontLeftMotor = new MotorEx("motor_esquerda").brakeMode().
            reversed();
    private MotorEx frontRightMotor = new MotorEx("motor_direita").brakeMode();
    private MotorEx backLeftMotor = new MotorEx("motor_esquerdatras").brakeMode().reversed();
    private MotorEx backRightMotor = new MotorEx("motor_direitatras").brakeMode();
    private IMUEx imu = new IMUEx("imu", Direction.LEFT, Direction.UP).zeroed();

    public next1() {
        addComponents(
                new SubsystemComponent(Shooter.INSTANCE, intake.INSTANCE, indexer.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

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

        button(() -> gamepad2.right_bumper)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() ->
                        new SequentialGroup(
                                intake.INSTANCE.shooting.and (indexer.INSTANCE.empurra).and (Shooter.INSTANCE.shootando),
                                Shooter.INSTANCE.shoot,
                                new Delay(0.6),
                                indexer.INSTANCE.puxa.and (intake.INSTANCE.shooting),
                                new Delay(0.6),
                                indexer.INSTANCE.puxa.and (intake.INSTANCE.shooting),
                                new Delay(0.6),
                                indexer.INSTANCE.puxa.and (intake.INSTANCE.shooting)

                        ).schedule()
                )
                .whenBecomesFalse(() ->
                        new SequentialGroup(
                                intake.INSTANCE.shooting.and (indexer.INSTANCE.empurra).and (Shooter.INSTANCE.shootando),
                                Shooter.INSTANCE.shoot,
                                new Delay(0.6),
                                indexer.INSTANCE.puxa.and (intake.INSTANCE.shooting),
                                new Delay(0.6),
                                indexer.INSTANCE.puxa.and (intake.INSTANCE.shooting),
                                new Delay(0.6),
                                indexer.INSTANCE.puxa.and (intake.INSTANCE.shooting)
                        ).schedule()
                );

        Gamepads.gamepad2().leftBumper().whenTrue(
                intake.INSTANCE.pega
                );

        button(() -> gamepad1.b)
                .whenBecomesTrue(() -> imu.getImu().resetYaw());

    }
}
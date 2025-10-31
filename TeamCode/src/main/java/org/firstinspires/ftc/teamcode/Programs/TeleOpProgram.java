package org.firstinspires.ftc.teamcode.Programs;

import static dev.nextftc.bindings.Bindings.button;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.indexer;
import org.firstinspires.ftc.teamcode.Subsystems.intake;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name = "TeleOp Field Oriented + NextFTC")
public class TeleOpProgram extends NextFTCOpMode {

    public TeleOpProgram() {
        addComponents(
                new SubsystemComponent(indexer.INSTANCE, intake.INSTANCE, Shooter.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private final MotorEx frontLeftMotor = new MotorEx("motor_esquerda").reversed();
    private final MotorEx frontRightMotor = new MotorEx("motor_direita");
    private final MotorEx backLeftMotor = new MotorEx("motor_esquerdatras").reversed();
    private final MotorEx backRightMotor = new MotorEx("motor_direitatras");

    private IMUEx imu = new IMUEx("imu", Direction.LEFT, Direction.UP).zeroed();

    @Override
    public void onStartButtonPressed() {

        button(() -> gamepad1.b)
                .whenBecomesTrue(() -> imu.getImu().resetYaw());

        button(() -> gamepad2.left_bumper)
                .whenTrue(() -> intake.INSTANCE.pega.schedule())
                .whenFalse(() -> intake.INSTANCE.stop.schedule());

        button(() -> gamepad2.right_bumper)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() ->
                        new SequentialGroup(
                                intake.INSTANCE.prepara.and(indexer.INSTANCE.empurra),
                                Shooter.INSTANCE.shoot,
                                new Delay(0.6),
                                intake.INSTANCE.shooting.and(indexer.INSTANCE.puxa),
                                intake.INSTANCE.shooting.and(indexer.INSTANCE.puxa),
                                intake.INSTANCE.shooting.and(indexer.INSTANCE.puxa)

                        ).schedule()
                );

        button(() -> gamepad2.a)
                .whenTrue(() -> indexer.INSTANCE.puxa.schedule())
                .whenFalse(() -> indexer.INSTANCE.para.schedule());
    }

    @Override
    public void onUpdate() {
        super.onUpdate();


    }
}

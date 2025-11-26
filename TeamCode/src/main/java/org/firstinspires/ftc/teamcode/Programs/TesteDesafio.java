package org.firstinspires.ftc.teamcode.Programs;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.Angulador;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.indexer;
import org.firstinspires.ftc.teamcode.Subsystems.intake;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp( name = "Prog Desafio" )
public class TesteDesafio extends NextFTCOpMode {
    public TesteDesafio() {
        addComponents(
                new SubsystemComponent(Shooter.INSTANCE, indexer.INSTANCE, intake.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE

        );
    }

    private final MotorEx frontLeftMotor = new MotorEx("motor_esquerda").reversed();
    private final MotorEx frontRightMotor = new MotorEx("motor_direita");
    private final MotorEx backLetMotor = new MotorEx("motor_esquerdatras").reversed();
    private final MotorEx backRightMotor = new MotorEx("motor_direitatras");
    private final MotorEx turret = new MotorEx("motor_turret");





    private IMUEx imu = new IMUEx("imu", Direction.LEFT, Direction.UP).zeroed();

    @Override
    public void onStartButtonPressed() {
        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLetMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX(),
                new FieldCentric(imu)

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

        if(gamepad2.right_trigger > 0.01){
            turret.setPower(0.3);
        } else if (gamepad2.left_trigger > 0.01){
            turret.setPower(-0.3);
        } else {
            turret.setPower(0);
        }

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

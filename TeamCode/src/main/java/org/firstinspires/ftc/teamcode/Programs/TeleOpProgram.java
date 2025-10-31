package org.firstinspires.ftc.teamcode.Programs;

import static dev.nextftc.bindings.Bindings.button;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.indexer;
import org.firstinspires.ftc.teamcode.Subsystems.intake;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
@Config
@TeleOp(name = "NextFTC TeleOp Program Java")
public class TeleOpProgram extends NextFTCOpMode {
    public TeleOpProgram() {
        addComponents(
                new SubsystemComponent(indexer.INSTANCE, intake.INSTANCE, Shooter.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }


    private static double i;
    private static double p;
    private static double d;
    public static PIDCoefficients coefficients = new PIDCoefficients(p, i, d);

    ControlSystem controlSystem = ControlSystem.builder()
            .posPid(coefficients)
            .build();

    private final MotorEx frontLeftMotor = new MotorEx("motor_esquerda").reversed();
    private final MotorEx frontRightMotor = new MotorEx("motor_direita");
    private final MotorEx backLeftMotor = new MotorEx("motor_esquerdatras").reversed();
    private final MotorEx backRightMotor = new MotorEx("motor_direitatras");
    private IMUEx imu = new IMUEx("imu", Direction.UP, Direction.FORWARD).zeroed();


    @Override
    public void onStartButtonPressed() {
        DriverControlledCommand driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX(),
                true
        );
        driverControlled.schedule();

        button(() -> gamepad1.b)
                .whenBecomesTrue(() -> imu.getImu().resetYaw());

        button(() -> gamepad2.right_bumper)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    Shooter.INSTANCE.shoot.schedule();
                    telemetry.addData("Shooter", "on");

                })
                .whenBecomesFalse(() -> {
                    Shooter.INSTANCE.parado.schedule();
                    telemetry.addData("Shooter", "off");
                });

        button(() -> gamepad2.left_bumper)
                .whenTrue(() -> {
                    intake.INSTANCE.pega.schedule();
                })
                .whenFalse(() -> {
                    intake.INSTANCE.stop.schedule();
                });

        button(() -> gamepad2.a)
                .whenTrue(() -> {
                    indexer.INSTANCE.puxa.schedule();
                })
                .whenFalse(() -> {
                    indexer.INSTANCE.para.schedule();
                });
    }


    @Override
    public void onUpdate() {
        super.onUpdate();

  }
}
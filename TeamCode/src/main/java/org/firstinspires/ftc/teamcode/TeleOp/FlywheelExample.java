
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
@Configurable
public class FlywheelExample extends OpMode {
    private DcMotorEx flywheelMotor;
    private ControlSystem controller;

    public static double Fkp = 0;
    public static double Fki = 0;
    public static double Fkd = 0;
    public static double Fks = 0;
    public static double Fka = 0;
    public static double Fkv = 0;

    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "f1");

        controller = ControlSystem.builder()
                .velPid(Fkp, Fki, Fkd)
                .basicFF(Fkv, Fka, Fks)
                .build();

        controller.setGoal(new KineticState(0.0, 0.0));
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            controller.setGoal(new KineticState(0.0, 2000.0));
        } else if (gamepad1.bWasPressed()) {
            controller.setGoal(new KineticState(0.0, 0.0));
        } else if (gamepad1.xWasPressed()) {
            controller.setGoal(new KineticState(0.0, 1000.0));
        }

        flywheelMotor.setPower(controller.calculate(new KineticState(
                flywheelMotor.getCurrentPosition(),
                flywheelMotor.getVelocity()))
        );
    }
}
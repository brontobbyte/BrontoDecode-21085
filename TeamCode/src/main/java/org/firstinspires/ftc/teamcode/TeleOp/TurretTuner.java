package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Subsystems.Turret.Tkp;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lock;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Configurable
@TeleOp(name = "Turret Tuner", group = "Tuning")
public class TurretTuner extends NextFTCOpMode {


    public TurretTuner () {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    public static double positionA = -60.0;
    public static double positionB = 60.0;

    private boolean prevX = false;
    private boolean prevB = false;

    @Override
    public void onInit() {

        Turret.INSTANCE.initialize();
        Turret.INSTANCE.reset();

        Turret.lockedAngle = positionB;
        Turret.destinationAngle = positionB;
        Turret.lockAngleEnabled = true;
        Turret.followEnabled = false;

        Turret.turretAngle = 0.0;
        Turret.toTurn = 0.0;
    }

    @Override
    public void onUpdate() {

        boolean curX = gamepad1.x;
        boolean curB = gamepad1.b;

        if (curX && !prevX) {
            Turret.lockedAngle = positionA;
            Turret.destinationAngle = positionA;
            Turret.lockAngleEnabled = true;
            Turret.followEnabled = false;
        }

        if (curB && !prevB) {
            Turret.lockedAngle = positionB;
            Turret.destinationAngle = positionB;
            Turret.lockAngleEnabled = true;
            Turret.followEnabled = false;
        }

        prevX = curX;
        prevB = curB;

        Turret.INSTANCE.periodic();
        telemetry.addData("Kp", Tkp);
        telemetry.addData("Posição X", "%.1f°", positionA);
        telemetry.addData("Posição B", "%.1f°", positionB);
        telemetry.addData("Ângulo Atual", "%.2f°", Turret.turretAngle);
        telemetry.addData("Setpoint", "%.2f°", Turret.destinationAngle);
        telemetry.addData("Erro", "%.00002f°", Turret.toTurn);
        telemetry.addData("Lock", Turret.lockAngleEnabled);
        telemetry.update();
    }
}
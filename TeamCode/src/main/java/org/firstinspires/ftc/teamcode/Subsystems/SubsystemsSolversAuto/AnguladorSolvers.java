package org.firstinspires.ftc.teamcode.Subsystems.SubsystemsSolversAuto;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

@Configurable
public class AnguladorSolvers extends SubsystemBase {

    private final Servo angulador;
    private final Servo angulador2;

    private static double angulo = 0.7;

    public AnguladorSolvers(final HardwareMap hMap, final String name) {
        angulador = hMap.get(Servo.class, "servo_angulador");

        angulador2 = hMap.get(Servo.class, "servo_angulador2");

    }

    /**
     * Alto.(+Perto)
     */
    public void On() {
        angulador.setPosition(0.95);
        angulador2.setPosition(0.05);
    }
    /**
     * Trava de intake
     */
    public void Off() {
        angulador.setPosition(0.35);
        angulador2.setPosition(0.65);
    }
    /**
     * Mínimo pra shootar
     */
    public void medio() {
        angulador.setPosition(angulo);
        angulador2.setPosition(1-angulo);
    }

}
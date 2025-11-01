package org.firstinspires.ftc.teamcode.Subsystems.SubsystemsSolversAuto;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class AnguladorSolvers extends SubsystemBase {

    private final Servo angulador;
    private final Servo angulador2;
    public AnguladorSolvers(final HardwareMap hMap, final String name) {
        angulador = hMap.get(Servo.class, "servo_angulador");

        angulador2 = hMap.get(Servo.class, "servo_angulador2");

    }

    /**
     * Alto.(+Perto)
     */
    public void On() {
        angulador.setPosition(0.25);
        angulador2.setPosition(0.75);
    }
    /**
     * Baixo.(+Longe)
     */
    public void Off() {
        angulador.setPosition(0.45);
        angulador2.setPosition(0.55);
    }
    /**
     * Baixo.(+Longe)
     */
    public void medio() {
        angulador.setPosition(-0.2);
        angulador2.setPosition(1.2);
    }

}
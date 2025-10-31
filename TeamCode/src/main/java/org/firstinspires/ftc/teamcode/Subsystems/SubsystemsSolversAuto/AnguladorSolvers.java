package org.firstinspires.ftc.teamcode.Subsystems.SubsystemsSolversAuto;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class AnguladorSolvers extends SubsystemBase {

    private final ServoEx angulador;
    private final ServoEx angulador2;
    public AnguladorSolvers(final HardwareMap hMap, final String name) {
        angulador = hMap.get(ServoEx.class, "servo_angulador");
        angulador2 = hMap.get(ServoEx.class, "servo_angulador2");

    }

    /**
     * Alto.(+Perto)
     */
    public void On() {
        angulador.set(-0.45);
        angulador2.set(0.45);
    }



    /**
     * Baixo.(+Longe)
     */
    public void Off() {
        angulador.set(0);
        angulador2.set(0);
    }

}
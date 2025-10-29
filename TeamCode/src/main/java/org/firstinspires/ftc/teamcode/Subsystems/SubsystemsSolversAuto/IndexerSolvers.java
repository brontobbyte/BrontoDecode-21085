package org.firstinspires.ftc.teamcode.Subsystems.SubsystemsSolversAuto;

import com.qualcomm.robotcore.hardware.CRServo;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IndexerSolvers extends SubsystemBase {

    private final CRServo mechRotation;

    public IndexerSolvers(final HardwareMap hMap, final String name) {
        mechRotation = hMap.get(CRServo.class, "servo_indexer");
    }

    /**
     * Manda pro Shooter.
     */
    public void On() {
        mechRotation.setPower(1);
    }



    /**
     * Desliga o Indexer.
     */
    public void Off() {
        mechRotation.setPower(0);
    }

}
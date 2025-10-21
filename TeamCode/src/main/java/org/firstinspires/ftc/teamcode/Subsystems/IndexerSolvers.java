package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
public class IndexerSolvers extends SubsystemBase {

    private final CRServo mechRotation;

    public IndexerSolvers(final HardwareMap hMap, final String name) {
        mechRotation = hMap.get(CRServo.class, "servo_indexer");
    }

    /**
     * Grabs a stone.
     */
    public void grab() {
        mechRotation.setPower(0.76);
    }



    /**
     * Releases a stone.
     */
    public void release() {
        mechRotation.setPower(0);
    }

}
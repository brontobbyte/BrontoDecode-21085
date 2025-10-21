package org.firstinspires.ftc.teamcode.Subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
public class IntakeSolvers extends SubsystemBase {

    private final MotorEx mechRotation;

    public IntakeSolvers(final HardwareMap hMap, final String name) {
        mechRotation = hMap.get(com.seattlesolvers.solverslib.hardware.motors.MotorEx.class, name);
    }

    /**
     * Grabs a stone.
     */
    public void grab() {
        mechRotation.setVelocity(0.76);
    }

    /**
     * Releases a stone.
     */
    public void release() {
        //mechRotation.setPosition(0);
    }

}
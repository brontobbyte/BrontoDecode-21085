package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
public class ShooterSolvers extends SubsystemBase {
    private Motor flywheel, flywheel2;
    private MotorGroup shooter;
    public ShooterSolvers(final HardwareMap hMap, final String name, final String name2) {
        flywheel = new Motor(hMap, name);
        flywheel2 = new Motor(hMap, name2);
        shooter = new MotorGroup(flywheel, flywheel2);
        //intake = hMap.get(com.seattlesolvers.solverslib.hardware.motors.Motor.class, "motor_direita");
    }

    /**
     * Grabs a stone.
     */
    public void grab() {
        shooter.setRunMode(Motor.RunMode.RawPower);
        shooter.set(1);
    }

    /**
     * Releases a stone.
     */
    public void release() {
        shooter.setRunMode(Motor.RunMode.RawPower);
        shooter.set(0);
        //mechRotation.setPosition(0);
    }

}
package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
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
    private Motor intake;
    public IntakeSolvers(final HardwareMap hMap, final String name) {
        intake = new Motor(hMap, name);
        //intake = hMap.get(com.seattlesolvers.solverslib.hardware.motors.Motor.class, "motor_direita");
    }

    /**
     * Grabs a stone.
     */
    public void grab() {
        intake.setRunMode(Motor.RunMode.RawPower);
        intake.set(0.6);
    }

    /**
     * Releases a stone.
     */
    public void release() {
        intake.setRunMode(Motor.RunMode.RawPower);
        intake.set(0);
        //mechRotation.setPosition(0);
    }

}
package org.firstinspires.ftc.teamcode.Subsystems.SubsystemsSolversAuto;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
     * Intake.
     */
    public void Intake() {
        intake.setRunMode(Motor.RunMode.RawPower);
        intake.set(-0.45);
    }
    /**
     * Velocidade para shootar
     */
    public void IntakeShoot() {
        intake.setRunMode(Motor.RunMode.RawPower);
        intake.set(-0.3);
    }

    /**
     * Para Intake.
     */
    public void Off() {
        intake.setRunMode(Motor.RunMode.RawPower);
        intake.set(0);
        //mechRotation.setPosition(0);
    }

}
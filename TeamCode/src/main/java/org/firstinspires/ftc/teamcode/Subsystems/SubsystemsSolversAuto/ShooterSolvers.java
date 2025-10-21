package org.firstinspires.ftc.teamcode.Subsystems.SubsystemsSolversAuto;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.sun.tools.javac.util.List;

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

    double corrected = shooter.getCorrectedVelocity();

    /**
     * Grabs a stone.
     */
    public void On() {
        shooter.setRunMode(Motor.RunMode.VelocityControl);
        shooter.setVeloCoefficients(1, 0, 0);
        shooter.setFeedforwardCoefficients(0, 1);
        //List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        //hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        shooter.set(1);
    }

    /**
     * Releases a stone.
     */
    public void Off() {
        shooter.setRunMode(Motor.RunMode.VelocityControl);
        shooter.setVeloCoefficients(1, 0, 0);
        shooter.setFeedforwardCoefficients(0, 1);
        //List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        //hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        shooter.set(1);
        //mechRotation.setPosition(0);
    }

}
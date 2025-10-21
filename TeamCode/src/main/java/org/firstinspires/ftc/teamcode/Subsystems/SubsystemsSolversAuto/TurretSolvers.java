package org.firstinspires.ftc.teamcode.Subsystems.SubsystemsSolversAuto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
public class TurretSolvers extends SubsystemBase {
    private Motor Turret;
    public TurretSolvers(final HardwareMap hMap, final String name) {
        Turret = new Motor(hMap, name);
    }
    /**
     * pos inical
     */
    public void ShootAuto(int target) {
        Turret.setRunMode(Motor.RunMode.PositionControl);
        Turret.setPositionCoefficient(0.05);
        Turret.setTargetPosition(target);
        //List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        //hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        Turret.set(0);
        while (!Turret.atTargetPosition()) {
            Turret.set(0.75);
        }
    }

}
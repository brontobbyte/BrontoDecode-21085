package org.firstinspires.ftc.teamcode.Subsystems.SubsystemsSolversAuto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
@Configurable
public class ShooterSolvers extends SubsystemBase {
    public static double kp = 1;
    public static double kv = 0;
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
    public void On() {
        /*shooter.setRunMode(Motor.RunMode.VelocityControl);
        shooter.setVeloCoefficients(kp, 0, 0);
        shooter.setFeedforwardCoefficients(0, kv);*/
        shooter.setRunMode(Motor.RunMode.RawPower);
        //List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        //hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        shooter.set(1);
        //telemetry.addData("vel", shooter.getVelocity());
        //telemetry.update();
    }

    /**
     * Releases a stone.
     */
    public void Off() {
        /*shooter.setRunMode(Motor.RunMode.VelocityControl);
        shooter.setVeloCoefficients(kp, 0, 0);
        shooter.setFeedforwardCoefficients(0, kv);*/
        shooter.setRunMode(Motor.RunMode.RawPower);
        //List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        //hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        shooter.set(0);
        //mechRotation.setPosition(0);
    }

}
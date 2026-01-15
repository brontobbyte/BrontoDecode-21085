package org.firstinspires.ftc.teamcode.Subsystems.SubsystemsSolversAuto;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
@Configurable
public class IntakeSolvers extends SubsystemBase {
    private Motor intake;
    public static double intakevel = 0.5
            ;

    public IntakeSolvers(final HardwareMap hMap, final String name) {
        intake = new Motor(hMap, name);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //intake = hMap.get(com.seattlesolvers.solverslib.hardware.motors.Motor.class, "motor_direita");
    }

    /**
     * Intake.
     */
    public void Intake() {
        intake.setRunMode(Motor.RunMode.RawPower);
        intake.set(1);
    }
    /**
     * Velocidade para shootar
     */
    public void IntakeShoot() {
        intake.setRunMode(Motor.RunMode.RawPower);
        intake.set(intakevel);
    }

    /**
     * Para Intake.
     */
    public void Off() {
        intake.setRunMode(Motor.RunMode.RawPower);
        intake.set(0);
        //mechRotation.setPosition(0);
    }
    public void vel(){
        intake.setRunMode(Motor.RunMode.VelocityControl);
        intake.setVeloCoefficients(1, 0, 0);
        intake.setFeedforwardCoefficients(0, 1);
        //shooter.setRunMode(Motor.RunMode.RawPower);
        //List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        //hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        intake.set(0.8);
    }

}
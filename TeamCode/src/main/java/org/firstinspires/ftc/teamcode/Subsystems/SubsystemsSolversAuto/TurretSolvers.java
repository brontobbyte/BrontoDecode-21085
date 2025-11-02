package org.firstinspires.ftc.teamcode.Subsystems.SubsystemsSolversAuto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

import java.util.List;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
@Configurable
public class TurretSolvers extends SubsystemBase {
    public static double kp = 0.018;
    public static double velo = 0.43;

    private Motor Turret;
    double targetDouble;
    int target;
    double Heading = 0;

    public TurretSolvers(final HardwareMap hMap, final String name) {
        Turret = new Motor(hMap, name);

    }
    /**
     * pos inical
     */
    public void ShootAuto(int target) {
        Turret.setRunMode(Motor.RunMode.PositionControl);
        Turret.setPositionTolerance(20);   // allowed maximum error
        Turret.setPositionCoefficient(kp);
        Turret.setTargetPosition(target);
        //List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        //hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        Turret.set(0);
        while (!Turret.atTargetPosition()) {
            Turret.set(velo);
        }
    }
    public void Off(){
        Turret.setRunMode(Motor.RunMode.RawPower);
        Turret.set(0);
    }

    @Override
    public void periodic() {

        // This method will be called once per scheduler run
    }
}
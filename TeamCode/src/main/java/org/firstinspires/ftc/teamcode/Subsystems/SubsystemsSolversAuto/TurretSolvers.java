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
    public static double kp = 0.002;
    private Motor Turret;
    double Heading = 0;
    Limelight3A limelight;

    public TurretSolvers(final HardwareMap hMap, final String name) {
        Turret = new Motor(hMap, name);
        limelight = hMap.get(Limelight3A.class, "limelight");
        //telemetry.setMsTransmissionInterval(11);
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(1); // Switch to pipeline number 0
        limelight.start();
    }
    /**
     * pos inical
     */
    public void ShootAuto(int target) {
        Turret.setRunMode(Motor.RunMode.PositionControl);
        Turret.setPositionTolerance(100);   // allowed maximum error
        Turret.setPositionCoefficient(kp);
        Turret.setTargetPosition(target);
        //List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        //hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        Turret.set(0);
        while (!Turret.atTargetPosition()) {
            Turret.set(-0.15);
        }
    }
    public void autoAlign(){
        Turret.setRunMode(Motor.RunMode.RawPower);
        Turret.set(Heading);
    }
    public void Off(){
        Turret.setRunMode(Motor.RunMode.RawPower);
        Turret.set(0);
    }

    @Override
    public void periodic() {
        LLResult result = limelight.getLatestResult();
        result.getPipelineIndex();
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                //telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                Heading = (-fr.getTargetXDegrees()/44);
            }
        }else{
            Heading = 0;
            //telemetry.addData("Limelight", "No data available");
        }
        // This method will be called once per scheduler run
    }
}
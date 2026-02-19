package org.firstinspires.ftc.teamcode.Constants;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
public class LimelightHelper {

    public static double updateAngleLL(Limelight3A limelight) {
        LLStatus status = limelight.getStatus();
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                return (-fr.getTargetXDegrees());
            }
        }
        return 0.0;
    }
    public static void addTelemetry(Telemetry telemetry, Limelight3A limelight, double angleLL) {
        LLStatus status = limelight.getStatus();
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f",
                        fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }
        } else {
            telemetry.addData("Limelight", "No data available");
        }
        telemetry.addData("angleLL", angleLL);
    }
}
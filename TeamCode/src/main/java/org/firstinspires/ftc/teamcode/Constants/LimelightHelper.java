package org.firstinspires.ftc.teamcode.Constants;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class LimelightHelper {

    private static boolean hasValidTarget = false;
    private static double lastValidAngle = 0;
    private static double filteredAngle = 0;

    private static final double smoothingFactor = 0.2;

    public static double updateAngleLL(Limelight3A limelight) {

        if (limelight == null) {
            hasValidTarget = false;
            return 0;
        }

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            hasValidTarget = false;
            return lastValidAngle;
        }

        List<LLResultTypes.FiducialResult> fiducialResults =
                result.getFiducialResults();

        if (fiducialResults == null || fiducialResults.isEmpty()) {
            hasValidTarget = false;
            return lastValidAngle;
        }

        LLResultTypes.FiducialResult fr = fiducialResults.get(0);

        double rawAngle = -fr.getTargetXDegrees();

        filteredAngle = (smoothingFactor * rawAngle)
                + ((1 - smoothingFactor) * filteredAngle);

        lastValidAngle = filteredAngle;
        hasValidTarget = true;

        return filteredAngle;
    }

    public static boolean hasTarget() {
        return hasValidTarget;
    }

    public static double getDistanceToTarget(Limelight3A limelight) {

        if (limelight == null) return 0;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return 0;

        List<LLResultTypes.FiducialResult> fiducialResults =
                result.getFiducialResults();

        if (fiducialResults == null || fiducialResults.isEmpty()) return 0;

        return fiducialResults.get(0)
                .getRobotPoseTargetSpace()
                .getPosition()
                .x;
    }

    public static void addTelemetry(Telemetry telemetry, Limelight3A limelight, double angleLL) {
        telemetry.addData("Limelight Has Target", hasValidTarget);
        telemetry.addData("Angle LL Filtered", "%.2f", angleLL);
    }
}
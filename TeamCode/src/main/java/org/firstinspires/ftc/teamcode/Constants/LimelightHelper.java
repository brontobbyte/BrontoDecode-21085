package org.firstinspires.ftc.teamcode.Constants;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

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
    public static Pose megaTag(Limelight3A limelight, Telemetry telemetry) {
        LLResult result = limelight.getLatestResult();
        double x = 0;
        double y = 0;
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                x = botpose.getPosition().x;
                y = botpose.getPosition().y;
                telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");
                return new Pose(x, y, 0, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
            }
        }else{
            return null;
        }
        return null;
    }
    public static boolean verifyLimelightPose(Pose newEstimate, double velo, double maxVelo) {
        if (!(newEstimate.getX() < 144 && newEstimate.getY() < 144 && newEstimate.getX() > 0 && newEstimate.getY() > 0)) {
            return false;
        }

        return !(Math.abs(velo) > maxVelo);
    }

}
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
    public static Pose getRobotPoseFromCamera(Limelight3A limelight, double heading) {
        double x = 0;
        double y = 0;
        //Fill this out to get the robot Pose from the camera's output (apply any filters if you need to using follower.getPose() for fusion)
        //Pedro Pathing has built-in KalmanFilter and LowPassFilter classes you can use for this
        //Use this to convert standard FTC coordinates to standard Pedro Pathing coordinates
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                x = botpose.getPosition().x;
                y = botpose.getPosition().y;
            }
        }
        if (x != 0 && y != 0) {
            return new Pose(x, y, heading, FTCCoordinates.INSTANCE);
        } else {
            return null;
        }
    }
}
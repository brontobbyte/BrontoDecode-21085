package org.firstinspires.ftc.teamcode.Constants;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryHelper {

    public static void addCommonTelemetry(
            Telemetry telemetry,
            Pose poseAtual,
            double flywheelVelocity,
            double turretAngle,
            double destinationAngle,
            double turretError,
            Limelight3A limelight,
            double angleLL,
            boolean debugMode,
            double distanceToGoal,
            double leftStickY,
            double leftStickX,
            double rightStickX
    ) {

        telemetry.addData("x", poseAtual.getX());
        telemetry.addData("y", poseAtual.getY());
        telemetry.addData("velo", flywheelVelocity);

        if (limelight != null) {
            LimelightHelper.addTelemetry(telemetry, limelight, angleLL);
        }

        if (debugMode) {
            telemetry.addData("turretAngle", turretAngle);
            telemetry.addData("destinationAngle", destinationAngle);
            telemetry.addData("turretError", turretError);
            telemetry.addData("distanceToGoal", distanceToGoal);
            telemetry.addData("flywheelVelocity", flywheelVelocity);
            telemetry.addData("Shooter Goal Distance", distanceToGoal);
            telemetry.addData("leftStickY", leftStickY);
            telemetry.addData("leftStickX", leftStickX);
            telemetry.addData("rightStickX", rightStickX);
        }
    }

    public static void addMinimalTelemetry(
            Telemetry telemetry,
            Pose poseAtual,
            double flywheelVelocity
    ) {
        telemetry.addData("heading", poseAtual.getHeading());
        telemetry.addData("x", poseAtual.getX());
        telemetry.addData("y", poseAtual.getY());
        telemetry.addData("velo", flywheelVelocity);
    }
}
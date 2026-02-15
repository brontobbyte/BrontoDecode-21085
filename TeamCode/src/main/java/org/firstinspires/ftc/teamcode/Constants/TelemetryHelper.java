package org.firstinspires.ftc.teamcode.Constants;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

import java.util.Objects;

public class TelemetryHelper {

    public static void addCommonTelemetry
            (Telemetry telemetry, Pose poseAtual, double flywheelVelocity,double turretAngle, double destinationAngle, double toTurn, Limelight3A limelight, double angleLL, boolean debugMode, double distanceToGoal, double leftStickY, double leftStickX, double rightStickX) {
        telemetry.addData("heading", poseAtual.getHeading());
        telemetry.addData("x", poseAtual.getX());
        telemetry.addData("y", poseAtual.getY());
        telemetry.addData("velo", flywheelVelocity);

        if (limelight != null) {
            LimelightHelper.addTelemetry(telemetry, limelight, angleLL);
        }

        if (debugMode) {
            telemetry.addData("turretAngle", turretAngle);
            telemetry.addData("destinationAngle", destinationAngle);
            telemetry.addData("toTurn", toTurn);
           // telemetry.addData("angleLL", Turret.angleLL);
            telemetry.addData("distanceToGoal", distanceToGoal);
            telemetry.addData("flywheelVelocity", flywheelVelocity);
            telemetry.addData("Shooter Goal Distance", distanceToGoal);
            telemetry.addData("leftStickY", leftStickY);
            telemetry.addData("leftStickX", leftStickX);
            telemetry.addData("rightStickX", rightStickX);
        }
    }

    public static void addMinimalTelemetry(Telemetry telemetry, Pose poseAtual, double flywheelVelocity) {
        telemetry.addData("heading", poseAtual.getHeading());
        telemetry.addData("x", poseAtual.getX());
        telemetry.addData("y", poseAtual.getY());
        telemetry.addData("velo", flywheelVelocity);
    }
}
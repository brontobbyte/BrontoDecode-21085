package org.firstinspires.ftc.teamcode.Programs;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@TeleOp
public class distancia extends OpMode {

    private Limelight3A limeLight3A;
    TestBench bench = new TestBench();
    private double distance;

    private double getTa;

    @Override
    public void init() {
        bench.init(hardwareMap);
        limeLight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limeLight3A.pipelineSwitch(1); // AprilTag 8 pipeline
    }

    @Override
    public void start() {
        limeLight3A.start();
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = bench.getOrientation();
        limeLight3A.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        LLResult llResult = limeLight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botpose = llResult.getBotpose_MT2();
            distance = getDistancfromtage(llResult.getTa());
            telemetry.addData("Distance", distance);
            telemetry.addData("Target X", llResult.getTx());
            telemetry.addData("Target Area", llResult.getTa());
        }
    }

    public double getDistancfromtage(double getTa) {

        double scale = 21884.19;
        double distance = (scale / getTa);
        return distance;
    }
}
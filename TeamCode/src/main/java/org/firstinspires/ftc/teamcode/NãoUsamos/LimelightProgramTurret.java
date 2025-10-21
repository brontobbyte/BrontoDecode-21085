package org.firstinspires.ftc.teamcode.NãoUsamos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
@Config
public class LimelightProgramTurret extends NextFTCOpMode {
    MotorEx Direita = new MotorEx("motor_direita");

    MotorEx DireitaTras = new MotorEx("motor_direitatras");

    MotorEx Esquerda = new MotorEx("motor_esquerda");
    MotorEx EsquerdaTras = new MotorEx("motor_esquerdatras");
    MotorEx motorTurret = new MotorEx("motor_turret");

    public static double HeadingGain = 3;
    private MotorEx frontLeftMotor = new MotorEx("motor_esquerda").
            reversed();
    private MotorEx frontRightMotor = new MotorEx("motor_direita");
    private MotorEx backLeftMotor = new MotorEx("motor_esquerdatras").
            reversed();
    private MotorEx backRightMotor = new MotorEx("motor_direitatras");
    private IMUEx imu = new IMUEx("imu", Direction.FORWARD, Direction.UP).zeroed();
    double Heading;
    double Strafe;
    double Drive;
    Limelight3A limelight;


    //FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void onInit() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(1); // Switch to pipeline number 0
        limelight.start();
        FtcDashboard.getInstance().startCameraStream(limelight, 90);

        //FtcDashboard.getInstance().startCameraStream(limelight, 0);
    }

    @Override
    public void onWaitForStart(){
        FtcDashboard.getInstance().startCameraStream(limelight, 90);
    }

    @Override
    public void onStartButtonPressed(){

    }
    @Override
    public void onUpdate() {
        double Heading2 = gamepad1.right_stick_x;
        Strafe = -gamepad1.left_stick_x;
        Drive = gamepad1.left_stick_y;
        new MotorEx(Esquerda.getMotor())
                .setPower(Heading2+(Strafe/2)+(Drive/2));
        new MotorEx(Direita.getMotor())
                .setPower(Heading2+(Strafe/2)-(Drive/2));
        new MotorEx(DireitaTras.getMotor())
                .setPower(-(Strafe/2)-(Drive/2));
        new MotorEx(EsquerdaTras.getMotor())
                .setPower(-(Strafe/2)+(Drive/2));

        TelemetryPacket packet = new TelemetryPacket();
        Telemetry telemetry1;
        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());
        telemetry.update();
        LLResult result = limelight.getLatestResult();
        result.getPipelineIndex();
        Pose3D botpose = result.getBotpose();

        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                Heading = (-fr.getTargetXDegrees()/22);

            }
        }else{
            telemetry.addData("Limelight", "No data available");
            Heading = 0;
        }
        new MotorEx(motorTurret.getMotor())
                .setPower(-Heading/HeadingGain);
        telemetry.addData("Strafe X", Strafe);
        telemetry.addData("Drive Y", Drive);
        telemetry.addData("Heading LL", Heading);
        telemetry.addData("Botpose", botpose.toString());
        packet.put("Botpose", botpose.toString());
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.sendTelemetryPacket(packet);


        //pose3D botpose = result.getBotPose();


    }

}
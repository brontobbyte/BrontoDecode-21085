package org.firstinspires.ftc.teamcode.Subsystems;



import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name= "limelightTeste")
public class LimelightMT2 extends LinearOpMode{



    private Limelight3A limelight;
    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException
    {
        imu = hardwareMap.get(IMU.class, "imu");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        IMU.Parameters parameters =
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                );

        imu.initialize(parameters);

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Inicia a coleta de dados.
         */
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();
            double robotYaw =
                    imu.getRobotYawPitchRollAngles()
                            .getYaw(AngleUnit.DEGREES);

            limelight.updateRobotOrientation(robotYaw);
            if (result != null && result.isValid()) {
                Pose3D botpose_mt2 = result.getBotpose_MT2();
                if (botpose_mt2 != null) {
                    double x = botpose_mt2.getPosition().x;
                    double y = botpose_mt2.getPosition().y;
                    telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
                    telemetry.update();
                }
            }
        }
    }
}
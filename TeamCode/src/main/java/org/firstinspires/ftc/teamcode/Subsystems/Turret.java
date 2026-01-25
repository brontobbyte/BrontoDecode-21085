package org.firstinspires.ftc.teamcode.Subsystems;
import static com.rowanmcalpin.nextftc.ftc.OpModeData.hardwareMap;

import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.Calculos.encoderTicksToAngle;
import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.Calculos.turnTurretBy;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants.AutoConstants;

import java.util.List;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.AngleType;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();

    private Turret() {
    }

    private static MotorEx motor = new MotorEx("motor_turret");
    private Limelight3A limelight;
    private static IMU imu;
    private final double ticks360 = 1000.0;
    private boolean limelightTracking = false;
    private final ControlSystem controlSystem = ControlSystem.builder()

            .angular(AngleType.DEGREES, feedback -> feedback.posPid(0.043, 0.000, 0.0))
            .build();

    private double targetDegrees = 0;

    public void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(4);
        limelight.start();
    }
    public void enableLimelightTracking() {
        limelightTracking = true;
    }
    public void disableLimelightTracking() {
        limelightTracking = false;
        motor.setPower(0);
    }
    public Limelight3A getLimelight() {
        return limelight;
    }
    public MotorEx getMotor() {
        return motor;
    }

    public double aimToObject(){
        double robotYPosition = 0/*odometria y*/, robotXPosition = 0/*odometria x*/;
        double destinationAngle = Math.atan2(132 - robotYPosition,
                137 - robotXPosition);

        double turretAngle = encoderTicksToAngle(motor.getRawTicks());
        double robotAngle = imu.getRobotYawPitchRollAngles().getYaw();

        double toTurn = destinationAngle - (turretAngle + robotAngle);
        return (toTurn%360);
         // take the mod/remainder of toTurn/360
        // to keep the angle in the range of [0,360]
    }
    public static Command TurretAlign = new LambdaCommand()
            .setStart(() -> {
                imu = hardwareMap.get(IMU.class, "imu");
                imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));

                // Runs on start
            })
            .setUpdate(() -> {
                double robotYPosition = 0, robotXPosition = 0;
                double destinationAngle = Math.atan2(132 - robotYPosition,
                        137 - robotXPosition);

                double turretAngle = encoderTicksToAngle(motor.getRawTicks());
                double robotAngle = imu.getRobotYawPitchRollAngles().getYaw();

                double toTurn = destinationAngle - (turretAngle + robotAngle);
                turnTurretBy(toTurn%360);
                // Runs on update
            })
            .setStop(interrupted -> {
                // Runs on stop
            })
            .setIsDone(() -> true) // Returns if the command has finished
            .requires(/* subsystems the command implements */)
            .setInterruptible(true)
            .named("My Command"); // sets the name of the command; optional


    @Override
    public void periodic() {
        if (limelightTracking && limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                if (!fiducialResults.isEmpty()) {
                    LLResultTypes.FiducialResult fr = fiducialResults.get(0);
                    double targetX = fr.getTargetXDegrees();

                    double erro =-targetX;

                    double currentDegrees = (motor.getCurrentPosition() / ticks360) * 360.0;

                    if ((currentDegrees >= 100 && erro > 0) || (currentDegrees <= -100 && erro < 0)) {
                        motor.setPower(0);
                        return;
                    }

                    double output = controlSystem.calculate(new KineticState(erro, 0, 0));
                    output = Range.clip(output, -1, 1);
                    motor.setPower(output);

                } else {
                    motor.setPower(0);
                }
            } else {
                motor.setPower(0);
            }
        } else {
            motor.setPower(0);
        }
        motor.setPower(turnTurretBy(aimToObject()));

    }
}
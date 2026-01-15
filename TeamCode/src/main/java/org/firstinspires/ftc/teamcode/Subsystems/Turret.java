package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.List;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.AngleType;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();

    private Turret() {
    }

    private MotorEx motor = new MotorEx("motor_turret");
    private Limelight3A limelight;
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
    }
}
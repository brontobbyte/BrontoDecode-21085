package org.firstinspires.ftc.teamcode.Programs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class TestBench {

    private BNO055IMU imu;


    public void init(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);
    }


    public YawPitchRollAngles getOrientation() {
        Orientation angles = imu.getAngularOrientation();
        return new YawPitchRollAngles(
                AngleUnit.DEGREES,
                angles.firstAngle,   // Yaw (Z)
                angles.secondAngle,  // Pitch (Y)
                angles.thirdAngle,   // Roll (X)
                0  // timestamp (não usado)
        );
    }
}

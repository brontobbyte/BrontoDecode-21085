package org.firstinspires.ftc.teamcode.Constants;

import static androidx.core.math.MathUtils.clamp;

import static org.firstinspires.ftc.teamcode.Subsystems.Turret.controllerauto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;

import com.qualcomm.robotcore.hardware.IMU;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class AutoConstants {
    private static Follower follower;
    @Configurable
    public static class Calculos {
        public static double pesoLL = 2;
        public static double pesoHeading = 1;
        public static double destinationAngleLL;
        private static MotorEx turretMotor = new MotorEx("turret");
        private static IMU imu;
        public static double scalingFactor = 0.9302325581;
        public static double encoderTicksToAngle(double ticks) {
            return (ticks * scalingFactor);
        }
        public static int angleToEncoderTicks(double degrees) {
            return (int) (degrees / scalingFactor);
        }
        public static double turnTurretBy(double degrees) {
            double currentPosition = turretMotor.getCurrentPosition();
            double destinationAngleHeading = angleToEncoderTicks(degrees);
            double TARGET_TICK_VALUE = clamp(destinationAngleHeading + currentPosition, angleToEncoderTicks(-180), angleToEncoderTicks(180));
            controllerauto.setGoal(new KineticState(TARGET_TICK_VALUE));
            return controllerauto.calculate(new KineticState(
                    turretMotor.getCurrentPosition(),
                    turretMotor.getVelocity()));
        }
    }
}
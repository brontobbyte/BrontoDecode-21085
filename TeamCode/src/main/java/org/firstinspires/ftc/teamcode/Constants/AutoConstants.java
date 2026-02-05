package org.firstinspires.ftc.teamcode.Constants;

import static androidx.core.math.MathUtils.clamp;

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
        private static ControlSystem controllerauto;

        public static double Tkp = 0.01;
        public static double Tki = 0;
        public static double Tkd = 0.0004;
        public static double pesoLL = 0;
        public static double pesoHeading = 1;
        public static double destinationAngleLL;
        private static MotorEx turretMotor = new MotorEx("turret");
        private static IMU imu;
        public static double scalingFactor = 0.1969365427;
        public static double encoderTicksToAngle(double ticks) {
            return (ticks * scalingFactor);
        }
        public static int angleToEncoderTicks(double degrees) {
            return (int) (degrees / scalingFactor);
        }
        public static double turnTurretBy(double degrees, double angleLL) {
            double currentPosition = turretMotor.getCurrentPosition();
            double destinationAngleHeading = angleToEncoderTicks(degrees);
            destinationAngleLL = angleToEncoderTicks(angleLL);
            double TARGET_TICK_VALUE = ((((destinationAngleHeading + destinationAngleLL)/2) + currentPosition)%angleToEncoderTicks(360));
            controllerauto = ControlSystem.builder()
                    .posPid(Tkp, Tki, Tkd)
                    .build();
            controllerauto.setGoal(new KineticState(TARGET_TICK_VALUE));
            return (controllerauto.calculate(new KineticState(
                    turretMotor.getCurrentPosition(),
                    turretMotor.getVelocity()))
            );

        }
    }
}

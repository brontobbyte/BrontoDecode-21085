package org.firstinspires.ftc.teamcode.Constants;

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
        private static ControlSystem controller;

        public static double Tkp = 0.01;
        public static double Tki = 0;
        public static double Tkd = 0.0002;
        public static double pesoLL = 5;
        public static double pesoHeading = 1;

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
            double destinationAngleLL = angleToEncoderTicks(angleLL);
            double destinationAngleHeading = angleToEncoderTicks(degrees);
            double TARGET_TICK_VALUE = (((destinationAngleHeading*pesoHeading) + (destinationAngleLL*pesoLL))/(pesoHeading+pesoLL)) + currentPosition;

            //turretMotor.setTargetPosition(TARGET_TICK_VALUE);
            //turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //turretMotor.setPower(1);
            controller = ControlSystem.builder()
                    .posPid(Tkp, Tki, Tkd)
                    .build();
            controller.setGoal(new KineticState(TARGET_TICK_VALUE));
            return (controller.calculate(new KineticState(
                    turretMotor.getCurrentPosition(),
                    turretMotor.getVelocity()))
            );

        }

    }




}

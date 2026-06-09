package org.firstinspires.ftc.teamcode.Constants;

import static androidx.core.math.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.controllerauto;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.control.KineticState;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class AutoConstants {

    @Configurable
    public static class Calculos {

        private static MotorEx turretMotor = new MotorEx("turret");
        public static double scalingFactor = 0.248275;

        public static double encoderTicksToAngle(double ticks) {
            return ticks * scalingFactor;
        }

        public static int angleToEncoderTicks(double degrees) {
            return (int) (degrees / scalingFactor);
        }

        public static double angleToEncoderTicksDouble(double degrees) {
            return degrees / scalingFactor;
        }

        public static double turnTurretBy(double degrees) {
            double currentPosition = turretMotor.getCurrentPosition();
            double destinationAngleHeading = angleToEncoderTicks(degrees);
            double TARGET_TICK_VALUE = clamp(
                    destinationAngleHeading + currentPosition,
                    angleToEncoderTicks(-270),
                    angleToEncoderTicks(90)
            );
            controllerauto.setGoal(new KineticState(TARGET_TICK_VALUE));
            return controllerauto.calculate(new KineticState(
                    turretMotor.getCurrentPosition(),
                    turretMotor.getVelocity()));
        }
    }
}
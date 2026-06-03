package org.firstinspires.ftc.teamcode.Constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class AutoConstants {

    @Configurable
    public static class Calculos {

        public static double scalingFactor = 0.9302325581;

        public static double max = 175.0;
        public static double min = -175.0;

        public static double encoderTicksToAngle(double ticks) {
            return ticks * scalingFactor;
        }

        public static int angleToEncoderTicks(double degrees) {
            return (int) (degrees / scalingFactor);
        }

        public static double angleToEncoderTicksDouble(double degrees) {
            return degrees / scalingFactor;
        }
    }
}
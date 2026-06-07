package org.firstinspires.ftc.teamcode.Constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;
@Configurable
public class autoshoot {
    public static double flywheeloffset = 0;
    public static double hoodOffset = 0.1;

    public static double flywheelSpeed(double goalDist) {
        double d = goalDist;
        double speed = -0.000231224 * Math.pow(d, 3)
                + 0.070295    * Math.pow(d, 2)
                - 0.412888    * d
                + 1162.72293;
        return MathFunctions.clamp(speed, 900, 2300) + flywheeloffset;
    }
    public static double hoodAngle(double goalDist) {
        double angle = 0.717069 * Math.sin(0.0142997 * goalDist - 2.24512) + 1.05582;
        return MathFunctions.clamp(angle, 0.33, 1) + hoodOffset;
    }
}
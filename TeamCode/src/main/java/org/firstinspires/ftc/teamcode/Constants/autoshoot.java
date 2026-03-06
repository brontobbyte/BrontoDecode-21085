package org.firstinspires.ftc.teamcode.Constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;
@Configurable
public class autoshoot {
    public static double flywheeloffset = 00;
    public static double hoodOffset = 0;
    public static double flywheelSpeed(double goalDist) {
        double speed =
                -0.0000213174 * Math.pow(goalDist, 4)
                        + 0.00696724 * Math.pow(goalDist, 3)
                        - 0.792929 * Math.pow(goalDist, 2)
                        + 43.47233 * goalDist
                        + 385.0875;

        return MathFunctions.clamp(speed, 900, 2300) + flywheeloffset;
    }
    public static double hoodAngle(double goalDist) {
        double angle =
                6.08358e-8 * Math.pow(goalDist, 4)
                        - 0.0000208664 * Math.pow(goalDist, 3)
                        + 0.00252368 * Math.pow(goalDist, 2)
                        - 0.120455 * goalDist
                        + 2.37088;

        return MathFunctions.clamp(angle, 0.05, 0.85) + hoodOffset;
    }
}
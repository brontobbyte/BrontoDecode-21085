package org.firstinspires.ftc.teamcode.Constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;
@Configurable
public class autoshoot {
    public static double flywheeloffset = 0;
    public static double hoodOffset = 0;
    public static double flywheelSpeed(double goalDist) {
        double speed =
                (0.00264319 * Math.pow(goalDist, 3))
                        - (0.55359 * Math.pow(goalDist, 2))
                        + (44.83383 * goalDist)
                        + 211.51441;

        return MathFunctions.clamp(speed, 1350, 2200) + flywheeloffset;
    }
    public static double hoodAngle(double goalDist) {
        double angle =
                (9.35134e-8 * Math.pow(goalDist, 4))
                        - (0.0000232553 * Math.pow(goalDist, 3))
                        + (0.00197687 * Math.pow(goalDist, 2))
                        - (0.0541954 * goalDist)
                        + 0.272586;

        return MathFunctions.clamp(angle, 0.05, 0.85) + hoodOffset;
    }
}
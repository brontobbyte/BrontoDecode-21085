package org.firstinspires.ftc.teamcode.Constants;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;

@Configurable
public class autoshoot {
    public static double flywheeloffset = 0;
    public static double hoodOffset = 0;
    public static double flywheelSpeed(double goalDist) {
        return MathFunctions.clamp(
                (((0.04582 * goalDist
                        - 6.3479) * goalDist
                        + 336.54) * goalDist
                        - 4832.61),
                0,
                1900
        ) + flywheeloffset;
    }
    public static double hoodAngle(double goalDist) {
        return MathFunctions.clamp(
                (((0.00001873 * goalDist
                        - 0.002531) * goalDist
                        + 0.13241) * goalDist
                        - 1.95652),
                0,
                0.85
        ) + hoodOffset;
    }
    public static double launchTime(double goalDist) {
        return 0.000018211 * Math.pow(goalDist, 2) - 0.00173368 * goalDist + 0.713636;
    }
}
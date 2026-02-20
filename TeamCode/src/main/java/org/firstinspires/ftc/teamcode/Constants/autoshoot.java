package org.firstinspires.ftc.teamcode.Constants;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;

@Configurable
public class autoshoot {
    public static double flywheeloffset = 0;
    public static double hoodOffset = 0;
    public static double flywheelSpeed(double goalDist) {
        return MathFunctions.clamp(
                0.0219148 * Math.pow(goalDist, 3) - 6.84911 * Math.pow(goalDist, 2) + 712.23515 * goalDist - 23096.5164,
                0,
                1900
        ) + flywheeloffset;
    }
    public static double hoodAngle(double goalDist) {
        return MathFunctions.clamp(
                -0.000002355408 * Math.pow(goalDist, 4) + 0.000945637 * Math.pow(goalDist, 3) - 0.1411955 * Math.pow(goalDist, 2) + 9.31183 * goalDist - 228.91371,
                0,
                0.85
        ) + hoodOffset;
    }
    public static double launchTime(double goalDist) {
        return 0.000018211 * Math.pow(goalDist, 2) - 0.00173368 * goalDist + 0.713636;
    }
}
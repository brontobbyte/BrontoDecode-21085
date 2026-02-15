package org.firstinspires.ftc.teamcode.Constants;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;

@Configurable
public class autoshoot {
    public static double flywheeloffset = 0;
    public static double hoodOffset = 0;
    public static double flywheelSpeed(double goalDist) {
        return MathFunctions.clamp(
                0.0259107 * Math.pow(goalDist, 2) + 0.879444 * goalDist + 619.2700, 0, 1400
        ) + flywheeloffset;
    }
    public static double hoodAngle(double goalDist) {
        return MathFunctions.clamp(
                0.0000912618 * Math.pow(goalDist, 2) - 0.0223322 * goalDist + 1.33875,
                0,
                0.6
        ) + hoodOffset;
    }
    public static double launchTime(double goalDist) {
        return 0.000018211 * Math.pow(goalDist, 2) - 0.00173368 * goalDist + 0.713636;
    }
}
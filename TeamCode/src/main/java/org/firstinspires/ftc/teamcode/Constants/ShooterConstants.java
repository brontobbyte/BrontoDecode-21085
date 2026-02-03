package org.firstinspires.ftc.teamcode.Constants;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;

@Configurable
public class ShooterConstants {
    public static double flywheeloffset = 0;

    public static double hoodOffset = 0;

    public static double flywheelSpeed(double goalDist) {
        return MathFunctions.clamp(
                -0.000119446 * Math.pow(goalDist, 4) + 0.0374223 * Math.pow(goalDist, 3) - 3.96361 * Math.pow(goalDist, 2) + 176.31759 * goalDist - 2468.71978, 0, 1500
        );
    }

    public static double hoodAngle(double goalDist) {
        return MathFunctions.clamp(
                9.47472e-8 * Math.pow(goalDist,4) - 0.0000340158 * Math.pow(goalDist, 3) + 0.00437277 * Math.pow(goalDist, 2) - 0.241482 * goalDist + 5.13899,
                0,
                0.6
        ) + hoodOffset;
    }

    public static double launchTime(double goalDist) {
        return 0.000018211 * Math.pow(goalDist, 2) - 0.00173368 * goalDist + 0.713636;
    }

    public static void flywheelOffset(double f) {
        flywheeloffset += f;
    }

    public static void hoodOffset(double h) {
        hoodOffset += h;
    }

    public static double getHoodOffset() {
        return hoodOffset;
    }

    public static double getFlywheelOffset() {
        return flywheeloffset;
    }
}
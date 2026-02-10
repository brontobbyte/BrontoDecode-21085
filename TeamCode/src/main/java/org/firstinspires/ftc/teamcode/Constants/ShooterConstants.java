package org.firstinspires.ftc.teamcode.Constants;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;

import org.opencv.core.Mat;

@Configurable
public class ShooterConstants {
    public static double flywheeloffset = 0;

    public static double hoodOffset = 0;
    public static double turretoffset = 0;


    public static double flywheelSpeed(double goalDist) {
        return MathFunctions.clamp(
                5.43014 * goalDist + 2843.13989, 0, 1700
        )+ flywheeloffset;
    }

    public static double hoodAngle(double goalDist) {
        return MathFunctions.clamp(
                0.000141062 * Math.pow(goalDist, 4) - 0.0413446 * Math.pow(goalDist, 3) + 4.02787 * Math.pow(goalDist, 2) - 151.42889 * goalDist + 2912.58376,
                0,
                0.6
        ) + hoodOffset;
    }

    public static double launchTime(double goalDist) {
        return (2.07987 * Math.pow(10, -7)) * Math.pow(goalDist, 4) - 0.0000593367 * Math.pow(goalDist, 3) + 0.00558008 * Math.pow(goalDist, 2) - 0.215845 * goalDist + 3.46671;
    }

    public static void flywheelOffset(double f) {
        flywheeloffset += f;
    }

    public static void hoodOffset(double h) {
        hoodOffset += h;
    }
    public static void turretOffset(double t) { turretoffset += t;}

    public static double getTurretOffset() { return turretoffset; }

    public static double getHoodOffset() {
        return hoodOffset;
    }

    public static double getFlywheelOffset() {
        return flywheeloffset;
    }
}
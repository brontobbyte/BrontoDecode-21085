package org.firstinspires.ftc.teamcode.Constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;
@Configurable
public class autoshoot {
    public static double flywheeloffset = 0;
    public static double hoodOffset = 0;

    public static double flywheelSpeed(double goalDist) {

        double speed =
                -0.0000803189 * Math.pow(goalDist, 4)
                        + 0.0227529 * Math.pow(goalDist, 3)
                        - 2.20277 * Math.pow(goalDist, 2)
                        + 91.10758 * goalDist;

        return MathFunctions.clamp(speed, 900, 2300) + flywheeloffset;
    }

    public static double hoodAngle(double goalDist) {

        double angle =
                0.00814601 * goalDist
                        + 0.043389;

        return MathFunctions.clamp(angle, 0.33, 1) + hoodOffset;
    }
}
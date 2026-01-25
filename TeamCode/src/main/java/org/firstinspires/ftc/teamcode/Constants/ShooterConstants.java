package org.firstinspires.ftc.teamcode.Constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;

@Configurable
public class ShooterConstants {
    public static double flywheelSpeed(double goalDist) {
        return MathFunctions.clamp(0/*Fórmula do site la*/, FlywheelConstants.flywheelMinimo, FlywheelConstants.flywheelMaximo);
    }

    public static double hoodAngle(double goalDist) {
        return MathFunctions.clamp(0/*Fórmula do site la*/, HoodConstants.hoodMinimo, HoodConstants.hoodMaximo);
    }
}

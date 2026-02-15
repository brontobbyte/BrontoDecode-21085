package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.Calculos.encoderTicksToAngle;
import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.Calculos.turnTurretBy;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@Configurable
public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();
    private double robotY;
    private double robotX;
    public static double angleLL;
    private double realAngleLL;
    private double lockedRealAngleLL; // Novo: para armazenar o valor corrigido quando alinhado
    private boolean isAligned = false; // Novo: flag para indicar se está alinhado

    public static double destinationAngle;
    public static double destinationAngleLL;

    private double heading;
    public static double toTurn;
    public static double goalx = 132;
    public static double goaly = 137; // vermei
    public static double turretAngle;
    public static double offset = -4;
    public static double div = 1.2;
    private static double originalDiv = 1.2;
    private static double minDiv = 0.1;
    private static double soma = 0.01;
    private static double treshold = 1.0;


    private boolean wrapped = false;

    private Turret() {
    }

    public void setPoseTracker(double robotX, double robotY, double heading, double angleLL) {
        this.robotX = robotX;
        this.robotY = robotY;
        this.heading = heading;
        this.angleLL = angleLL;
    }
    private static final MotorEx motor = new MotorEx("turret");

    @Override
    public void initialize() {
    }
    public void reset() {
        motor.zeroed();
        wrapped = false;
        isAligned = false;
    }
    public double aimToObject() {
        double robotYPosition = robotY, robotXPosition = robotX;
        destinationAngle = Math.toDegrees(Math.atan2(goaly - robotYPosition, goalx - robotXPosition));

        if (Math.abs(angleLL) <= treshold) {
            if (!isAligned) {
                lockedRealAngleLL = realAngleLL;
                isAligned = true;
            }
            div = originalDiv;
        } else {
            isAligned = false;
            if (Math.abs(angleLL) > 2) {
                div = Math.max(minDiv, div - soma);
            } else {
                div = originalDiv;
            }
        }
        if (angleLL == 0){
            div = originalDiv;
        }

        if (isAligned) {
            realAngleLL = lockedRealAngleLL;
        } else {
            if (angleLL != 0.0) {
                realAngleLL = angleLL / div + offset;
            }
        }

        double correctedDestinationAngle = destinationAngle;
        if (realAngleLL != 0.0) {
            correctedDestinationAngle = destinationAngle + realAngleLL;
        }
        turretAngle = encoderTicksToAngle(motor.getRawTicks());
        double robotAngle = heading;

        toTurn = correctedDestinationAngle - (turretAngle + robotAngle);
        toTurn = ((toTurn + 180) % 360) - 180;

        if (turretAngle > 170 || (toTurn > 0 && turretAngle + toTurn > 170)) {
            toTurn -= 360;
            wrapped = true;
        } else if (turretAngle < -170 || (toTurn < 0 && turretAngle + toTurn < -170)) {
            toTurn += 360;
            wrapped = true;
        } else {
            wrapped = false;
        }

        return toTurn;
    }

    @Override
    public void periodic() {
        motor.setPower(turnTurretBy(aimToObject(), angleLL));
    }
}
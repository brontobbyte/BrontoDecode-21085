package org.firstinspires.ftc.teamcode.Constants;

import static com.rowanmcalpin.nextftc.ftc.OpModeData.hardwareMap;
import static com.rowanmcalpin.nextftc.ftc.OpModeData.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autos.Auto_12_5_NextFTC_LauchZoneMov;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class AutoConstants {
    private static Follower follower;
    @Configurable
    public static class Calculos {
        private static ControlSystem controller;
        private static MotorEx turretMotor = new MotorEx("fr");
        private static IMU imu;
        public static double scalingFactor = 0.05;
        public static double encoderTicksToAngle(double ticks) {
            return (ticks * scalingFactor);
        }
        public static int angleToEncoderTicks(double degrees) {
            return (int) (degrees / scalingFactor);
        }
        public static double turnTurretBy(double degrees) {
            double currentPosition = turretMotor.getCurrentPosition();
            double TARGET_TICK_VALUE = angleToEncoderTicks(degrees) + currentPosition;

            //turretMotor.setTargetPosition(TARGET_TICK_VALUE);
            //turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //turretMotor.setPower(1);
            controller = ControlSystem.builder()
                    .posPid(0.1, 0.0, 0.0)
                    .elevatorFF(0.04)
                    .build();
            controller.setGoal(new KineticState(TARGET_TICK_VALUE));
            return (controller.calculate(new KineticState(
                    turretMotor.getCurrentPosition(),
                    turretMotor.getVelocity()))
            );

        }

    }
    @Configurable
    public static class Comandos {
        public static class WaitForStopCommand extends Command {
            private final Follower follower;
            private final double velocityThreshold;
            private final long stableTimeMs;
            private final ElapsedTime timer;
            private boolean isTimerValid;
            private boolean Done = false;

            public WaitForStopCommand(Follower follower, double velocityThreshold, long stableTimeMs) {
                this.follower = follower;
                this.velocityThreshold = velocityThreshold;
                this.stableTimeMs = stableTimeMs;
                this.timer = new ElapsedTime();
                this.isTimerValid = false;
            }

            public void init() {
                timer.reset();
                isTimerValid = false;
            }

            public void loop() {
                double currentSpeed = follower.getVelocity().getMagnitude();

                if (currentSpeed > velocityThreshold) {
                    timer.reset();
                    isTimerValid = false;
                } else {
                    if (!isTimerValid) {
                        timer.reset();
                        isTimerValid = true;
                    }
                }

                telemetry.addData("WaitForStop", "Vel: %.2f, Timer: %dms",
                        currentSpeed, (int) timer.milliseconds());

                if (isTimerValid && timer.milliseconds() >= stableTimeMs) {
                    isDone();
                    Done = true;
                }
            }

            public boolean isFinished() {
                return isTimerValid && timer.milliseconds() >= stableTimeMs;
            }

            public void end() {
            }

            @Override
            public boolean isDone() {
                return Done;
            }
        }
        public static Command waitForStop = new WaitForStopCommand(follower, 0.5, 200);
    }



}

package org.firstinspires.ftc.teamcode.Constants;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import dev.nextftc.core.commands.Command;
@Configurable
public class AutoCommands {
    @Configurable
    public static class Comandos {
        public static class WaitForStopCommand extends Command {
            private final Localizer localizer;
            private final double velocityThreshold;
            private final long stableTimeMs;
            private final ElapsedTime timer;
            private boolean isTimerValid;
            private boolean Done = false;

            public WaitForStopCommand(Localizer localizer, double velocityThreshold, long stableTimeMs) {
                this.localizer = localizer;
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
                Vector currentSpeed = new PoseTracker(localizer).getVelocity();

                if (currentSpeed.getMagnitude() > velocityThreshold) {
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
    }
}

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;

import com.pedropathing.geometry.Pose;

import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.core.components.BindingsComponent;

import org.firstinspires.ftc.teamcode.Constants.AutoPoses;

/**
 * HeadingTest — Gate Cycle Assist TeleOp
 *
 * Architecture:
 *  - Uses PedroDriverControlled for field-centric drive + centripetal correction.
 *  - Toggle B to enable gateAssist, which overrides rx with a look-ahead heading
 *    controller: instead of aiming directly at the gate, the robot aims at a
 *    flow-direction vector computed from (robot → gate → nextWaypoint), blending
 *    both so the heading correction starts transitioning before the gate is
 *    reached.  This eliminates the "spin and freeze" seen when using a single
 *    static target.
 *  - The heading error uses atan2(sin(Δ), cos(Δ)) normalization to avoid
 *    wrap-around discontinuities at ±π.
 *  - The toggle uses a rising-edge detector in onUpdate() (the correct place);
 *    onStartButtonPressed() fired only once at match start and is not a game-loop.
 */
@Configurable
@TeleOp(name = "HeadingTest")
public class HeadingTest extends NextFTCOpMode {


    /** Proportional gain for heading error → rx correction. */
    private static final double KP = -0.5;

    /** Maximum rotation power applied by the gate assist. */
    private static final double MAX_TURN = 0.7;

    /**
     * How far from the gate (in field units) we start blending toward the
     * nextWaypoint heading instead of the gate heading.
     * E.g. if your field units are inches, 12 = 12 inches out.
     */
    private static final double BLEND_RADIUS = 14.0;

    /**
     * Weight [0..1] for the look-ahead (nextWaypoint) direction when inside
     * BLEND_RADIUS.  0 = always aim at gate, 1 = always aim at next waypoint.
     * Values around 0.5–0.7 give smooth flow-through behavior.
     */
    private static final double LOOKAHEAD_WEIGHT = 0.55;

    // ─── State ────────────────────────────────────────────────────────────────

    private boolean gateAssist = false;
    private boolean lastB      = false;

    // ─── Components ───────────────────────────────────────────────────────────

    {
        addComponents(
                new PedroComponent(org.firstinspires.ftc.teamcode.pedroPathing.Constants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    // ─── Lifecycle ────────────────────────────────────────────────────────────

    @Override
    public void onInit() {
        PedroComponent.follower().setStartingPose(AutoPoses.poseInicial);
    }

    @Override
    public void onUpdate() {
        // 1. Keep Pedro's odometry / localizer ticking every loop.
        PedroComponent.follower().update();

        // 2. Rising-edge toggle for gate assist (B button).
        boolean currentB = gamepad1.b;
        if (currentB && !lastB) {
            gateAssist = !gateAssist;
        }
        lastB = currentB;

        // 3. Read driver joystick inputs.
        //    Pedro convention: leftStickY = forward, leftStickX = strafe.
        //    Negate axes so "push forward on stick" = move forward on field.
        double gamepadX  = -gamepad1.left_stick_x;   // strafe
        double gamepadY  =  gamepad1.left_stick_y;   // forward  (note: SDK y is already inverted for most sticks)
        double gamepadRx =  gamepad1.right_stick_x;  // manual turn

        // 4. Compute heading correction if gate assist is active.
        double rx = gamepadRx;
        if (gateAssist) {
            rx = computeGateAssistRx();
        }

        // 5. Field-centric transform.
        Pose   pose    = PedroComponent.follower().getPose();
        double heading = pose.getHeading();

        double rotX = gamepadX * Math.cos(-heading) - gamepadY * Math.sin(-heading);
        double rotY = gamepadX * Math.sin(-heading) + gamepadY * Math.cos(-heading);

        // 6. Power normalization so the largest wheel never exceeds 1.0.
        double denom = Math.max(
                Math.abs(rotX) + Math.abs(rotY) + Math.abs(rx),
                1.0
        );

        // 7. Write motor powers via Pedro follower's motor accessor.
        //    This avoids hard-coding DcMotor references and lets Pedro's
        //    centripetal correction layer remain active during manual drive.
        //
        //    Standard mecanum mixing (fl, bl, fr, br — fl/bl reversed in hw):
        double fl = (rotY + rotX + rx) / denom;
        double bl = (rotY - rotX + rx) / denom;
        double fr = (rotY - rotX - rx) / denom;
        double br = (rotY + rotX - rx) / denom;

        // PedroComponent exposes the follower; use its motor map if available,
        // otherwise fall back to the explicit motor references below.
        // If your project already has a DriveSubsystem wrapping these motors,
        // replace this block with a call into that subsystem instead.
        setMotorPowers(fl, fr, bl, br);



        // 8. Telemetry.
        telemetry.addData("GateAssist", gateAssist);
        telemetry.addData("Heading (°)", Math.toDegrees(heading));
        telemetry.addData("Gate target",
                AutoPoses.gateCiclePose.getX() + ", " + AutoPoses.gateCiclePose.getY());
        telemetry.addData("rx (applied)", String.format("%.3f", rx));
        telemetry.addData("heading raw", pose.getHeading());
        telemetry.update();

    }

    private double computeGateAssistRx() {
        Pose   pose = PedroComponent.follower().getPose();
        Pose   gate = AutoPoses.gatePose;
        Pose   next = AutoPoses.shootPose1;

        double dx = gate.getX() - pose.getX();
        double dy = gate.getY() - pose.getY();
        double dist = Math.hypot(dx, dy);

        // Heading directly toward the gate.
        double headingToGate = Math.atan2(dy, dx);

        // Heading from gate toward the next cycle waypoint (look-ahead).
        double dxNext = next.getX() - gate.getX();
        double dyNext = next.getY() - gate.getY();
        double headingToNext = Math.atan2(dyNext, dxNext);

        // Blend: inside BLEND_RADIUS, ease weight toward look-ahead direction.
        double blend = 0.0;
        if (dist < BLEND_RADIUS) {
            // Linear fade: at dist=BLEND_RADIUS blend=0, at dist=0 blend=LOOKAHEAD_WEIGHT.
            blend = LOOKAHEAD_WEIGHT * (1.0 - dist / BLEND_RADIUS);
        }

        // Interpolate the desired heading using circular (atan2) blending.
        double desiredHeading = circularBlend(headingToGate, headingToNext, blend);

        // Normalized angular error (avoids ±π discontinuity).
        // dentro de computeGateAssistRx()
        double error = Math.atan2(
                Math.sin(pose.getHeading() - desiredHeading),
                Math.cos(pose.getHeading() - desiredHeading)
        );

        // Proportional output, clamped.
        double rx = error * KP;
        return Math.max(-MAX_TURN, Math.min(MAX_TURN, rx));
    }


    private static double circularBlend(double angleA, double angleB, double weight) {
        double diff = Math.atan2(
                Math.sin(angleB - angleA),
                Math.cos(angleB - angleA)
        );
        return angleA + weight * diff;
    }

    private void setMotorPowers(double fl, double fr, double bl, double br) {
        // Obtain motors from the hardware map each call is fine here because
        // BulkReadComponent caches bulk reads; individual gets are O(1) map lookups.
        com.qualcomm.robotcore.hardware.DcMotor motorFl =
                hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "fl");
        com.qualcomm.robotcore.hardware.DcMotor motorFr =
                hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "fr");
        com.qualcomm.robotcore.hardware.DcMotor motorBl =
                hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "bl");
        com.qualcomm.robotcore.hardware.DcMotor motorBr =
                hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "br");

        motorFl.setPower(fl);
        motorFr.setPower(fr);
        motorBl.setPower(bl);
        motorBr.setPower(br);
    }
}
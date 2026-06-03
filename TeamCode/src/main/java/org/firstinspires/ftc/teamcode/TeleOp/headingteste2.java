package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants.AutoPoses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Configurable
@TeleOp(name = "headingteste2")
public class headingteste2 extends NextFTCOpMode {

    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;

    public static double targetHeadingDeg = 150;

    public static double kpHeading = 0.1;
    public static double maxTurnPower = 0.09;

    private boolean headingLocked = false;

    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {

        FrontLeft = hardwareMap.get(DcMotor.class, "fl");
        FrontRight = hardwareMap.get(DcMotor.class, "fr");
        BackLeft = hardwareMap.get(DcMotor.class, "bl");
        BackRight = hardwareMap.get(DcMotor.class, "br");

        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);

        PedroComponent.follower().setStartingPose(
                AutoPoses.poseInicial
        );
    }

    @Override
    public void onStartButtonPressed() {

        Gamepads.gamepad1().b().whenBecomesTrue(() ->
                headingLocked = !headingLocked
        );
    }

    @Override
    public void onUpdate() {

        PedroComponent.follower().update();

        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double heading = PedroComponent.follower()
                .getPose()
                .getHeading();

        if (headingLocked) {

            double targetRad =
                    Math.toRadians(targetHeadingDeg + 180);

            double error = Math.atan2(
                    Math.sin(targetRad - heading),
                    Math.cos(targetRad - heading)
            );

            double errorDeg = Math.abs(
                    Math.toDegrees(error)
            );

            // Dá prioridade ao giro quando está muito longe
            if (errorDeg > 45) {

                x *= 0.4;
                y *= 0.4;

            } else if (errorDeg > 25) {

                x *= 0.6;
                y *= 0.6;

            } else if (errorDeg > 10) {

                x *= 0.8;
                y *= 0.8;
            }

            rx = error * kpHeading;

            rx = Math.max(
                    -maxTurnPower,
                    Math.min(maxTurnPower, rx)
            );

            if (errorDeg < 2.0) {
                rx = 0;
            }
        }

        double rotX =
                x * Math.cos(-heading)
                        - y * Math.sin(-heading);

        double rotY =
                x * Math.sin(-heading)
                        + y * Math.cos(-heading);

        rotX *= 1.1;

        double denominator = Math.max(
                Math.abs(rotY)
                        + Math.abs(rotX)
                        + Math.abs(rx),
                1.0
        );

        double fl = (rotY + rotX + rx) / denominator;
        double bl = (rotY - rotX + rx) / denominator;
        double fr = (rotY - rotX - rx) / denominator;
        double br = (rotY + rotX - rx) / denominator;

        FrontLeft.setPower(fl);
        BackLeft.setPower(bl);
        FrontRight.setPower(fr);
        BackRight.setPower(br);

        telemetry.addData(
                "Heading (deg)",
                Math.toDegrees(heading)
        );

        telemetry.addData(
                "Target Heading",
                targetHeadingDeg
        );

        telemetry.addData(
                "Heading Lock",
                headingLocked
        );

        telemetry.addData(
                "Heading Error",
                Math.toDegrees(
                        Math.atan2(
                                Math.sin(Math.toRadians(targetHeadingDeg + 180) - heading),
                                Math.cos(Math.toRadians(targetHeadingDeg + 180) - heading)
                        )
                )
        );

        telemetry.addData(
                "Turn Power",
                rx
        );

        telemetry.update();
    }
}
package org.firstinspires.ftc.teamcode.pedroPathing;
import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.poseInicial;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants.AutoPoses;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lock;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import java.util.function.Supplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
@TeleOp
public class TeleOpPose extends NextFTCOpMode {
    public TeleOpPose() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE),
                new SubsystemComponent(Lock.INSTANCE),
                new SubsystemComponent(Shooter.INSTANCE),
                new SubsystemComponent(Hood.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private Follower follower;
    public static Pose startingPose = AutoPoses.poseInicial; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    public Pose shoot = new Pose (60, 10, Math.toRadians(180));

    @Override
    public void onInit() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(92, 95),
                                shoot
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    @Override
    public void onStartButtonPressed() {
        Command driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX().negate(),
                false

        );
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void onUpdate() {

        MotorEx motor = new MotorEx("turret");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Call this once per loop
        follower.update();
        telemetryM.update();
        Shooter.INSTANCE.periodic();
        if (!automatedDrive) {
            if (gamepad1.a) {
                follower.followPath(pathChain.get());
                automatedDrive = true;
            }

            //Stop automated following if the follower is done
            if (automatedDrive && (gamepad1.b || !follower.isBusy())) {
                follower.startTeleopDrive();
                automatedDrive = false;
            }

            //Slow Mode
            if (gamepad1.right_bumper) {
                slowMode = !slowMode;
            }

            //Optional way to change slow mode strength
            if (gamepad1.x) {
                slowModeMultiplier += 0.25;
            }

            //Optional way to change slow mode strength
            if (gamepad1.y) {
                slowModeMultiplier -= 0.25;
            }
            if (gamepad1.dpad_right)  {
                ShooterConstants.turretOffset(-40);
                sleep(100);
            }
            if (gamepad1.dpad_left) {
                ShooterConstants.turretOffset(40);
                sleep(100);
            }
            if (gamepad1.a) {
                ShooterConstants.hoodOffset(0.1);
                new Delay(0.5);
            }
            if (gamepad1.b) {
                ShooterConstants.hoodOffset(-0.1);
                new Delay(0.5);
            }
            if (gamepad1.x) {
                ShooterConstants.flywheelOffset(100);
                sleep(100);
            }
            if (gamepad1.y) {
                ShooterConstants.flywheelOffset(-100);
                sleep(100);
            }
            if (gamepad1.start) {
                PedroComponent.follower().setPose(shoot);
            }
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }

        //Automated PathFollowing



    }
}
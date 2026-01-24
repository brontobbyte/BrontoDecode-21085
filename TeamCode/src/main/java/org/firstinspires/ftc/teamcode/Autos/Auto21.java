package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.localization.Localizer;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous
public class Auto21 extends NextFTCOpMode {
    private MotorEx turretMotor = new MotorEx("fr");
    public double scalingFactor = 0.05;
    private ControlSystem controller;
    private Follower follower;
    private Localizer localizer;
    private IMU imu;
    private HardwareMap hardwareMap;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    {
        addComponents(/* vararg components */);
    }


    public double encoderTicksToAngle(double ticks) {
        return (ticks * scalingFactor);
    }

    public int angleToEncoderTicks(double degrees) {
        return (int) (degrees / scalingFactor);
    }

    public void turnTurretBy(double degrees) {
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
        turretMotor.setPower(controller.calculate(new KineticState(
                turretMotor.getCurrentPosition(),
                turretMotor.getVelocity()))
        );
    }

    Command TurretAlign = new LambdaCommand()
            .setStart(() -> {
                imu = hardwareMap.get(IMU.class, "imu");
                imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));

                // Runs on start
            })
            .setUpdate(() -> {
                double robotYPosition = 0, robotXPosition = 0;
                double destinationAngle = Math.atan2(132 - robotYPosition,
                        137 - robotXPosition);

                double turretAngle = encoderTicksToAngle(turretMotor.getRawTicks());
                double robotAngle = imu.getRobotYawPitchRollAngles().getYaw();

                double toTurn = destinationAngle - (turretAngle + robotAngle);
                turnTurretBy(toTurn%360);
                // Runs on update
            })
            .setStop(interrupted -> {
                // Runs on stop
            })
            .setIsDone(() -> true) // Returns if the command has finished
            .requires(/* subsystems the command implements */)
            .setInterruptible(true)
            .named("My Command"); // sets the name of the command; optional

    @Override public void onInit() {
    }
    @Override public void onWaitForStart() { }
    @Override public void onStartButtonPressed() {

        CommandManager.INSTANCE.scheduleCommand(TurretAlign);
    }
    @Override public void onUpdate() { }
    @Override public void onStop() { }
}


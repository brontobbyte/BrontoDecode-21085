package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.Calculos.angleToEncoderTicks;
import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.Calculos.encoderTicksToAngle;
import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.Comandos.waitForStop;
import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.poseInicial;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.Constants.AutoConstants;
import org.firstinspires.ftc.teamcode.Constants.AutoPaths;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//TODO AUTONOMOUS - 21 ARTIFACTS CLASSIFIED + 3 BASE - 21085 - BRONTOBYTE - BR
@Autonomous
public class Auto21 extends NextFTCOpMode {
    public double scalingFactor = 0.05;
    private Follower follower;
    private Localizer localizer;
    private PathChain InicialIntake, Shoot1, Shoot2, Shoot3, Gate, ShootDoGate, Intake2, Intake3;

    {
        addComponents(new SubsystemComponent(Turret.INSTANCE));
    }
    public void buildPaths() {
        InicialIntake = AutoPaths.InicialIntake;
        Shoot1        =        AutoPaths.Shoot1;
        Gate          =          AutoPaths.Gate;
        ShootDoGate   =   AutoPaths.ShootDoGate;
        Intake2       =       AutoPaths.Intake2;
        Shoot2        =        AutoPaths.Shoot2;
        Intake3       =       AutoPaths.Intake3;
        Shoot3        =        AutoPaths.Shoot3;
    }
    @Override public void onInit() {
        telemetry.update();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(poseInicial);
    }
    @Override public void onWaitForStart() { }
    @Override public void onStartButtonPressed() {
        CommandManager.INSTANCE.scheduleCommand(Turret.TurretAlign);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(poseInicial);
        buildPaths();
        CommandManager.INSTANCE.scheduleCommand(
                new ParallelGroup(
                        CommandManager.INSTANCE.scheduleCommand(

                        ),
                        new SequentialGroup(
                                new FollowPath(InicialIntake),
                                new FollowPath(Shoot1),
                                new ParallelDeadlineGroup(
                                        waitForStop,
                                        new FollowPath(Gate)
                                        // vararg Command
                                ),
                                new FollowPath(ShootDoGate),
                                new ParallelDeadlineGroup(
                                        waitForStop,
                                        new FollowPath(Gate)
                                        // vararg Command
                                ),
                                new FollowPath(ShootDoGate),
                                new ParallelDeadlineGroup(
                                        waitForStop,
                                        new FollowPath(Gate)
                                        // vararg Command
                                ),
                                new FollowPath(ShootDoGate),
                                new FollowPath(Intake2),
                                new FollowPath(Shoot2),
                                new FollowPath(Intake3),
                                new FollowPath(Shoot3)
                        )
                )
        );
        follower.update();
    }
    @Override public void onUpdate() { }
    @Override public void onStop() { }
}


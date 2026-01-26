package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.poseInicial;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.localization.localizers.ThreeWheelIMULocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.callbacks.ParametricCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Constants.AutoCommands;
import org.firstinspires.ftc.teamcode.Constants.AutoPaths;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lock;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//TODO AUTONOMOUS - 21 ARTIFACTS CLASSIFIED + 3 BASE - 21085 - BRONTOBYTE - BR
@Autonomous
public class Auto21 extends NextFTCOpMode{
    {
        addComponents(
                new SubsystemComponent(Turret.INSTANCE/*, Shooter.INSTANCE*/),
                new PedroComponent(Constants::createFollower)
        );
    }
    private Follower follower;
    private Localizer localizer;
    private HardwareMap hardwareMap;
    private PathChain InicialIntake, Shoot1, Shoot2, Shoot3, Gate, ShootDoGate, Intake2, Intake3;
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
        PedroComponent.follower().setStartingPose(poseInicial);
        Pose poseAtual = PedroComponent.follower().getPose();
        Turret.INSTANCE.setPoseTracker(poseAtual.getX(), poseAtual.getY(), poseAtual.getHeading());
        //Shooter.INSTANCE.setPoseTracker(poseAtual);

    }
    @Override public void onWaitForStart() {
        Pose poseAtual = PedroComponent.follower().getPose();
        Turret.INSTANCE.setPoseTracker(poseAtual.getX(), poseAtual.getY(), Math.toDegrees(poseAtual.getHeading()));
        Turret.INSTANCE.periodic();
        telemetry.addData("heading", poseAtual.getHeading());
        telemetry.addData("x", poseAtual.getX());
        telemetry.update();

    }
    @Override public void onStartButtonPressed() {
        //follower = Constants.createFollower(hardwareMap);
        //follower.setStartingPose(poseInicial);
        //buildPaths();
        CommandManager.INSTANCE.scheduleCommand(new Delay(30000));
        /*CommandManager.INSTANCE.scheduleCommand(
                new ParallelGroup(
                        new SequentialGroup(
                                new ParallelGroup(
                                        new FollowPath(InicialIntake),
                                        Lock.INSTANCE.open,
                                        Intake.INSTANCE.shooting.afterTime(1)
                                ),
                                new ParallelGroup(
                                        new FollowPath(Shoot1),
                                        Lock.INSTANCE.open
                                ),
                                Intake.INSTANCE.shooting,
                                new ParallelDeadlineGroup(
                                        new AutoCommands.Comandos.WaitForStopCommand(localizer, 5, 200),
                                        new FollowPath(Gate, true, 0.4).setInterruptible(true)
                                        // vararg Command
                                ),
                                new Delay(1000),
                                new ParallelGroup(
                                        new FollowPath(ShootDoGate),
                                        Lock.INSTANCE.open
                                ),
                                Intake.INSTANCE.shooting,
                                new ParallelDeadlineGroup(
                                        new AutoCommands.Comandos.WaitForStopCommand(localizer, 5, 200),
                                        new FollowPath(Gate, true, 0.4).setInterruptible(true)
                                        // vararg Command
                                ),
                                new Delay(1000),
                                new ParallelGroup(
                                        new FollowPath(ShootDoGate),
                                        Lock.INSTANCE.open
                                ),
                                Intake.INSTANCE.shooting,
                                new ParallelDeadlineGroup(
                                        new AutoCommands.Comandos.WaitForStopCommand(localizer, 5, 100),
                                        new FollowPath(Gate, true, 0.4).setInterruptible(true)
                                        // vararg Command
                                ),
                                new Delay(1000),
                                new ParallelGroup(
                                        new FollowPath(ShootDoGate),
                                        Lock.INSTANCE.open
                                ),
                                Intake.INSTANCE.shooting,
                                new FollowPath(Intake2),
                                new ParallelGroup(
                                        new FollowPath(Shoot2),
                                        Lock.INSTANCE.open
                                ),
                                Intake.INSTANCE.shooting,
                                new FollowPath(Intake3),
                                new ParallelGroup(
                                        new FollowPath(Shoot3),
                                        Lock.INSTANCE.open
                                ),
                                Intake.INSTANCE.shooting
                        )
                )
        );
        follower.update();*/
    }
    @Override public void onUpdate() {
        Pose poseAtual = PedroComponent.follower().poseTracker.getPose();
        Turret.INSTANCE.setPoseTracker(poseAtual.getX(), poseAtual.getY(), Math.toDegrees(poseAtual.getHeading()));
        Turret.INSTANCE.periodic();
        telemetry.addData("heading", poseAtual.getHeading());
        telemetry.addData("y", poseAtual.getY());
        telemetry.addData("x", poseAtual.getX());
        telemetry.update();
    }
    @Override public void onStop() { }
}


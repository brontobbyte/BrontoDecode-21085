package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.Constants.AutoCommands.Comandos.waitForStop;
import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.poseInicial;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.ftc.NextFTCOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
public class Auto21 extends NextFTCOpMode {
    private Follower follower;
    private PathChain InicialIntake, Shoot1, Shoot2, Shoot3, Gate, ShootDoGate, Intake2, Intake3;

    {
        addComponents(new SubsystemComponent(Turret.INSTANCE, Hood.INSTANCE, Shooter.INSTANCE));
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
                        new SequentialGroup(
                                new ParallelGroup(
                                        new FollowPath(InicialIntake),
                                        Lock.INSTANCE.open,
                                        Intake.INSTANCE.shooting.afterTime(1)
                                ),
                                new FollowPath(Shoot1),
                                new ParallelDeadlineGroup(
                                        new AutoCommands.Comandos.WaitForStopCommand(follower, 5, 100),
                                        new FollowPath(Gate, true, 0.4).setInterruptible(true)
                                        
                                        // vararg Command
                                ),
                                new Delay(1000),
                                new FollowPath(ShootDoGate),
                                new ParallelDeadlineGroup(
                                        new AutoCommands.Comandos.WaitForStopCommand(follower, 5, 100),
                                        new FollowPath(Gate, true, 0.4).setInterruptible(true)
                                        // vararg Command
                                ),
                                new Delay(1000),
                                new FollowPath(ShootDoGate),
                                new ParallelDeadlineGroup(
                                        new AutoCommands.Comandos.WaitForStopCommand(follower, 5, 100),
                                        new FollowPath(Gate, true, 0.4).setInterruptible(true)
                                        // vararg Command
                                ),
                                new Delay(1000),
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


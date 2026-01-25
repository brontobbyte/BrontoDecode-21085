package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.Comandos.waitForStop;
import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.poseInicial;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.ftc.NextFTCOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants.AutoPaths;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//TODO AUTONOMOUS - 21 ARTIFACTS CLASSIFIED + 3 BASE - 21085 - BRONTOBYTE - BR
@Autonomous(name="Auto21", group="Autonomous")
public class Auto21 extends NextFTCOpMode {

    private PathChain InicialIntake, Shoot1, Shoot2, Shoot3, Gate, ShootDoGate, Intake2, Intake3;
    private Follower follower;

    public void buildPaths() {
        InicialIntake = AutoPaths.InicialIntake;
        Shoot1        = AutoPaths.Shoot1;
        Gate          = AutoPaths.Gate;
        ShootDoGate   = AutoPaths.ShootDoGate;
        Intake2       = AutoPaths.Intake2;
        Shoot2        = AutoPaths.Shoot2;
        Intake3       = AutoPaths.Intake3;
        Shoot3        = AutoPaths.Shoot3;
    }

    @Override
    public void onInit() {
        telemetry.update();
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setPose(poseInicial);
        buildPaths();
    }

    @Override
    public void onWaitForStart() { }

    @Override
    public void onStartButtonPressed() {

        CommandManager.INSTANCE.scheduleCommand(
                new ParallelGroup(
                        Turret.TurretAlign,
                        new SequentialGroup(
                                new FollowPath(InicialIntake),
                                new FollowPath(Shoot1),
                                new ParallelDeadlineGroup(
                                        waitForStop,
                                        new FollowPath(Gate)
                                ),
                                new FollowPath(ShootDoGate),
                                new ParallelDeadlineGroup(
                                        waitForStop,
                                        new FollowPath(Gate)
                                ),
                                new FollowPath(ShootDoGate),
                                new ParallelDeadlineGroup(
                                        waitForStop,
                                        new FollowPath(Gate)
                                ),
                                new FollowPath(ShootDoGate),
                                new FollowPath(Intake2),
                                new FollowPath(Shoot2),
                                new FollowPath(Intake3),
                                new FollowPath(Shoot3)
                        )
                )
        );
    }

    @Override
    public void onUpdate() { }

    @Override
    public void onStop() { }
}

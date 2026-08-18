package org.firstinspires.ftc.teamcode.Autos;
import static com.pedropathing.math.MathFunctions.clamp;
import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.goalPoseazul;
import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.goalShootPoseAzul;
import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.poseInicial;
import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.shootPose1;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.Tkd;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.Tki;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.toTurn;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.turretAngle;
import static org.firstinspires.ftc.teamcode.TeleOp.TeleOpAzul.projectileSpeed;
import static org.firstinspires.ftc.teamcode.TeleOp.TeleOpAzul.projectileSpeedAutoX;
import static org.firstinspires.ftc.teamcode.TeleOp.TeleOpAzul.projectileSpeedAutoY;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.Constants.AutoConstants;
import org.firstinspires.ftc.teamcode.Constants.AutoPathsVermelho;
import org.firstinspires.ftc.teamcode.Constants.AutoPathsVermelho;
import org.firstinspires.ftc.teamcode.Constants.AutoPoses;
import org.firstinspires.ftc.teamcode.Constants.PoseManager;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lock;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Constants.LimelightHelper;
import org.firstinspires.ftc.teamcode.Constants.TelemetryHelper;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
// TODO AUTONOMOUS - 21 ARTIFACTS CLASSIFIED AZUL + 3 BASE - 21085 - BRONTOBYTE - BR
@Configurable
@Autonomous
public class Auto21Vermelho extends NextFTCOpMode {
    {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Lock.INSTANCE, Hood.INSTANCE),
                new SubsystemComponent(Shooter.INSTANCE, Indexer.INSTANCE, Turret.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }
    public static double Fkp = 0.005;
    public static double Fki = 0.000000001;
    public static double Fkd = 0;
    public static double Fks = 0.17;
    public static double Fka = 6;
    public static double Fkv = 0.00030;
    private final MotorGroup Flywheel = new MotorGroup(
            new MotorEx("f1"),
            new MotorEx("f2")
    );
    public static boolean debugMode = true;
    public static double distanceToGoal;
    public static double kp = 0.12;
    public static double goal = 1400;
    public static double tol = 1.5;
    public static double gatedelay = 1;
    public static double shootdist = 0.99;
    public static double carolina = 0.8;
    public static double filter = 0;
    public static double turretTolerance = 1;

    public static boolean preloadActive = true;
    public static double preloadFlywheelOffset = 0;
    private static ControlSystem controller;
    private double angleLL = 0;
    @Override
    public void onInit() {
        //Intake.INSTANCE.initialize();
        Turret.INSTANCE.resetTurret();
        Intake.INSTANCE.stop.invoke();
        Lock.INSTANCE.closed.invoke();
        Lock.INSTANCE.open.invoke();
        Hood.INSTANCE.set.invoke();
        Indexer.INSTANCE.naoshooting.invoke();
        Shooter.INSTANCE.setVelocity(0);
        PedroComponent.follower().setStartingPose(poseInicial.mirror());
        CommandManager.INSTANCE.scheduleCommand(Lock.INSTANCE.closed);
        Pose poseAtual = PedroComponent.follower().poseTracker.getPose();
        Turret.INSTANCE.setPoseTracker(poseAtual.getX(), poseAtual.getY(), poseAtual.getHeading(), 0, false, telemetry, 0);

        new AutoPathsVermelho();
    }
    @Override
    public void onWaitForStart() {
        Pose poseAtual = PedroComponent.follower().poseTracker.getPose();
        double distanceToGoal = PedroComponent.follower().getPose().distanceFrom(goalShootPoseAzul.mirror());
        //Shooter.INSTANCE.setGoalDistance(0);
        telemetry.update();
    }
    @Override
    public void onStartButtonPressed() {
        Turret.enabled = true;
        Intake.INSTANCE.intake.invoke();
        CommandManager.INSTANCE.scheduleCommand(
                new SequentialGroup(
                        preload(),
                        new WaitUntil(() -> {
                            preloadActive = false;
                            return true;
                        }),
                        intakeMeio(),
                        shootMeio(),
                        gateCicle(),
                        gateCicle(),
                        intakeCima(),
                        shootCima(),
                        intakeBaixo(),
                        shootfinal(),
                        gateCicle(),
                        new FollowPath(AutoPathsVermelho.last(PedroComponent.follower()))
                )
        );
        PedroComponent.follower().update();
    }
    @Override
    public void onUpdate() {
        PedroComponent.follower().update();
        Pose poseAtual = PedroComponent.follower().getPose();
        distanceToGoal = PedroComponent.follower().getPose().distanceFrom(goalShootPoseAzul.mirror());
        double targetVelocity =
                Shooter.INSTANCE.getGoalForDistance(distanceToGoal);
        if (preloadActive) {
            targetVelocity += preloadFlywheelOffset;
        }
        Shooter.INSTANCE.setVelocity(goal);
        Shooter.INSTANCE.periodic();
        Hood.INSTANCE.setHoodPos(Hood.pos);
        Hood.INSTANCE.periodic();
        double lateralVel = PedroComponent.follower().poseTracker.getVelocity().getYComponent();
        double vertVel = PedroComponent.follower().poseTracker.getVelocity().getXComponent();
        double flightTimey = distanceToGoal / projectileSpeedAutoY;
        double flightTimex = distanceToGoal / projectileSpeedAutoX;
        double leadCompensation = flightTimex * vertVel + flightTimey * lateralVel;

        Turret.INSTANCE.setPoseTracker(shootPose1.mirror().getX(), shootPose1.mirror().getY(), shootPose1.mirror().getHeading(), 0, false, telemetry, 0);
        telemetry.update();
    }
    @Override
    public void onStop() {
        PoseManager.currentPose = PedroComponent.follower().getPose();
    }
    private SequentialGroup shootar() {
        return new SequentialGroup(
                new WaitUntil(() ->
                        PedroComponent.follower().poseTracker.getVelocity().getMagnitude() < 5.0
                ),
                new Delay(0.10),
                Lock.INSTANCE.open,
                Indexer.INSTANCE.shooting,
                Intake.INSTANCE.intake,
                new Delay(0.50),
                Indexer.INSTANCE.naoshooting,
                Lock.INSTANCE.closed
        );
    }
    private SequentialGroup shootpathdist(PathChain path) {
        return new SequentialGroup(
//                new ParallelGroup(
                new FollowPath(path),
                new SequentialGroup(
//                                new WaitUntil(() ->
//                                        PedroComponent.follower()
//                                                .getCurrentPath()
//                                                .getClosestPointTValue() > shootdist &&
//                                                Math.abs(Turret.toTurn) < turretTolerance
//                                ),
                        Indexer.INSTANCE.shooting,
                        shootar()
                )
                //)
        );
    }
    private SequentialGroup intake() {
        return new SequentialGroup(
                Indexer.INSTANCE.naoshooting,
                Lock.INSTANCE.closed,
                Intake.INSTANCE.intake
        );
    }
    private SequentialGroup stopintake() {
        return new SequentialGroup(
                Intake.INSTANCE.stop
        );
    }
    private SequentialGroup gateCicle() {
        return new SequentialGroup(
                new ParallelGroup(
                        intake().afterTime(1),
                        new FollowPath(
                                AutoPathsVermelho.Gate(PedroComponent.follower()),
                                true,
                                0.98
                        )
                ),
                new FollowPath(
                        AutoPathsVermelho.Gate2(PedroComponent.follower()),
                        true,
                        0.55
                ),
                new Delay(gatedelay),
                intake(),
                shootpathdist(
                        AutoPathsVermelho.ShootGate(PedroComponent.follower())
                )
        );
    }

    private SequentialGroup shootMeio() {
        return shootpathdist(
                AutoPathsVermelho.ShootMeio(PedroComponent.follower())
        );
    }
    private SequentialGroup intakeMeio() {
        return new SequentialGroup(
                intake(),
                new FollowPath(AutoPathsVermelho.IntakeMeio(PedroComponent.follower())),
                intake()
        );
    }
    private SequentialGroup preload() {
        preloadActive = true;
        Hood.INSTANCE.setHoodPos(Hood.pos);
        return new SequentialGroup(
                shootpathdist(
                        AutoPathsVermelho.ShootPreload(PedroComponent.follower())
                ),
                Indexer.INSTANCE.shooting,
                intake(),
                shootar()
        );
    }
    private SequentialGroup intakeCima() {
        return new SequentialGroup(
                new ParallelGroup(
                        intake().afterTime(0.5),
                        new FollowPath(AutoPathsVermelho.IntakeCima(PedroComponent.follower()))
                ),
                intake()
        );
    }
    private SequentialGroup shootCima() {
        return new SequentialGroup(
                intake(),
                new FollowPath(AutoPathsVermelho.ShootCima(PedroComponent.follower())),
                new WaitUntil(() -> Math.abs(Turret.toTurn) < turretTolerance),
                Lock.INSTANCE.open,
                shootar()
        );
    }
    private SequentialGroup intakeBaixo() {
        return new SequentialGroup(
                new ParallelGroup(
                        intake().afterTime(0.8),
                        new FollowPath(AutoPathsVermelho.IntakeBaixo(PedroComponent.follower()))
                ),
                intake()
        );
    }
    private SequentialGroup shootfinal() {
        return shootpathdist(
                AutoPathsVermelho.shootPoselast(PedroComponent.follower())
        );
    }

    public class EnableShootBrake extends Command {

        @Override
        public void start() {
            Turret.enableShootBrake();
        }

        @Override
        public boolean isDone() {
            return true;
        }
    }

    public class DisableShootBrake extends Command {

        @Override
        public void start() {
            Turret.disableShootBrake();
        }

        @Override
        public boolean isDone() {
            return true;
        }
    }
}
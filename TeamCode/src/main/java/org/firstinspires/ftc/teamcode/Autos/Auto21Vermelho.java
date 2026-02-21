package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.goalPoseVermelho;
import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.goalPoseazul;
import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.poseInicial;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.toTurn;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.turretAngle;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Constants.AutoPathsVermelho;
import org.firstinspires.ftc.teamcode.Constants.PoseManager;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
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
                new SubsystemComponent(Shooter.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }
    private static final MotorEx motor = new MotorEx("turret").brakeMode();

    public static boolean debugMode = true;
    public static double distanceToGoal;

    Limelight3A limelight;
    private double angleLL = 0;

    @Override
    public void onInit() {
        Intake.INSTANCE.initialize();
        Intake.INSTANCE.intake.invoke();
        Intake.INSTANCE.stop.invoke();
        Lock.INSTANCE.closed.invoke();
        Lock.INSTANCE.open.invoke();
        Shooter.INSTANCE.getPower();
        angleLL = 0;
        PedroComponent.follower().setStartingPose(poseInicial.mirror());
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(400);
        limelight.pipelineSwitch(4);
        limelight.start();
        CommandManager.INSTANCE.scheduleCommand(Lock.INSTANCE.open);
        new AutoPathsVermelho();
    }

    @Override
    public void onWaitForStart() {
        angleLL = LimelightHelper.updateAngleLL(limelight);
        Pose poseAtual = PedroComponent.follower().poseTracker.getPose();
        Turret.INSTANCE.setPoseTracker(poseAtual.getX(), poseAtual.getY(), Math.toDegrees(poseAtual.getHeading()), angleLL, false, telemetry);
        double distanceToGoal = PedroComponent.follower().getPose().distanceFrom(goalPoseVermelho);
        //Shooter.INSTANCE.setGoalDistance(distanceToGoal);

        TelemetryHelper.addCommonTelemetry(telemetry, PedroComponent.follower().getPose(),
                Shooter.INSTANCE.getVelocity(), turretAngle, Turret.destinationAngle, toTurn,
                limelight, angleLL, debugMode, distanceToGoal, 0.0, 0.0, 0.0);
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        PedroComponent.follower().setStartingPose(poseInicial.mirror());
        CommandManager.INSTANCE.scheduleCommand(
                new SequentialGroup(
                        Lock.INSTANCE.open,
                        preload(),
                        intakeMeio(),
                        new FollowPath(AutoPathsVermelho.Gate(PedroComponent.follower())),
                        shootMeio(),
                        shootar(),
                        new Delay(0.6),
                        intakeCima(),
                        shootCima(),
                        new Delay(0.6),
                        intakeBaixo(),
                        shootfinal(),
                        new Delay(0.6),
                        new FollowPath(AutoPathsVermelho.last(PedroComponent.follower()))

                )
        );
        PedroComponent.follower().update();
    }

    @Override
    public void onUpdate() {
        PedroComponent.follower().update();
        angleLL = LimelightHelper.updateAngleLL(limelight);
        Pose poseAtual = PedroComponent.follower().poseTracker.getPose();
        //Turret.INSTANCE.setPoseTracker(poseAtual.getX(), poseAtual.getY(), Math.toDegrees(poseAtual.getHeading()), angleLL, false, telemetry);
        distanceToGoal = PedroComponent.follower().getPose().distanceFrom(goalPoseVermelho);
        Shooter.INSTANCE.setGoalDistance(distanceToGoal);
        Shooter.INSTANCE.periodic();
        Hood.INSTANCE.setGoalDistance(distanceToGoal);
        Hood.INSTANCE.periodic();
        TelemetryHelper.addCommonTelemetry(telemetry, PedroComponent.follower().getPose(),
                Shooter.INSTANCE.getVelocity(), turretAngle, Turret.destinationAngle, toTurn,
                limelight, angleLL, debugMode, distanceToGoal, 0.0, 0.0, 0.0);
        telemetry.update();
    }

    @Override
    public void onStop() {
        Shooter.INSTANCE.setGoalDistance(0);
        PoseManager.currentPose = PedroComponent.follower().getPose();
    }

    private SequentialGroup shootar() {
        return new SequentialGroup(
                Lock.INSTANCE.open,
                Intake.INSTANCE.intake,
                new Delay(0.5),
                Intake.INSTANCE.stop,
                new Delay(0.2),
                Intake.INSTANCE.intake,
                new Delay(0.3),
                Intake.INSTANCE.stop

                );
    }
    private SequentialGroup intake() {
        return new SequentialGroup(
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
                new FollowPath(AutoPathsVermelho.Gate(PedroComponent.follower())),
                intake(),
                new FollowPath(AutoPathsVermelho.GateCicle(PedroComponent.follower())),
                //new FollowPath(AutoPathsVermelho.GateCicleFinal(PedroComponent.follower())),
                stopintake(),
                new FollowPath(AutoPathsVermelho.ShootGate(PedroComponent.follower())),
                shootar()
        );
    }
    private SequentialGroup gateCicleCima() {
        return new SequentialGroup(
                new FollowPath(AutoPathsVermelho.Gate(PedroComponent.follower())),
                intake(),
                new FollowPath(AutoPathsVermelho.GateCicle(PedroComponent.follower())),
                //new FollowPath(AutoPathsVermelho.GateCicleFinal(PedroComponent.follower())),
                stopintake(),
                new FollowPath(AutoPathsVermelho.ShootGateCima(PedroComponent.follower())),
                shootar()
        );
    }
    private SequentialGroup shootMeio() {
        return new SequentialGroup(
                new FollowPath(AutoPathsVermelho.ShootMeio(PedroComponent.follower())),
                shootar()
        );
    }
    private SequentialGroup intakeMeio(){
        return new SequentialGroup(
                intake(),
                new FollowPath(AutoPathsVermelho.IntakeMeio(PedroComponent.follower())),   //Vai pra fileira do meio
                stopintake()
        );
    }
    private SequentialGroup preload(){
        return new SequentialGroup(
                new FollowPath(AutoPathsVermelho.ShootPreload(PedroComponent.follower())), //Shoot Preload
                shootar()
        );
    }
    private SequentialGroup intakeCima(){
        return new SequentialGroup(
                intake(),
                new FollowPath(AutoPathsVermelho.IntakeCima(PedroComponent.follower())),
                stopintake()
        );
    }
    private SequentialGroup shootCima(){
        return new SequentialGroup(
                new FollowPath(AutoPathsVermelho.ShootCima(PedroComponent.follower())),
                shootar()
        );
    }
    private SequentialGroup intakeBaixo(){
        return new SequentialGroup(
                intake(),
                new FollowPath(AutoPathsVermelho.IntakeBaixo(PedroComponent.follower())),
                stopintake()
        );
    }
    private SequentialGroup shootfinal(){
        return new SequentialGroup(
                new FollowPath(AutoPathsVermelho.shootPoselast(PedroComponent.follower())),
                shootar()
        );
    }
}
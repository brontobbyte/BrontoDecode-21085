package org.firstinspires.ftc.teamcode.Autos;
import static com.pedropathing.math.MathFunctions.clamp;
import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.goalPoseazul;
import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.poseInicial;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.Tkd;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.Tki;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.toTurn;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.turretAngle;
import static org.firstinspires.ftc.teamcode.TeleOp.TeleOpAzul.projectileSpeed;
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
import org.firstinspires.ftc.teamcode.Constants.AutoPathsAzul;
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
public class Auto21Azul extends NextFTCOpMode {
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
    public static double goal = 1620;
    public static double tol = 1.5;
    public static double gatedelay = 1.6;

    public static double distforshoot = 6.0;
    public static double carolina = 0.8;
    public static double filter = 0;
    private static ControlSystem controller;
    //    Limelight3A limelight;
    private double angleLL = 0;
    @Override
    public void onInit() {
        Hood.INSTANCE.setHoodPos(Hood.pos);
        Intake.INSTANCE.initialize();
        Intake.INSTANCE.intake.invoke();
        Intake.INSTANCE.stop.invoke();
        Lock.INSTANCE.closed.invoke();
        Lock.INSTANCE.open.invoke();
        Hood.INSTANCE.set.invoke();
        Indexer.INSTANCE.naoshooting.invoke();
        Shooter.INSTANCE.setGoalDistance(0);
        // Shooter.INSTANCE.getPower();
        PedroComponent.follower().setStartingPose(poseInicial);
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.setPollRateHz(400);
//        limelight.pipelineSwitch(5);
//        limelight.start();
        CommandManager.INSTANCE.scheduleCommand(Lock.INSTANCE.closed);
        new AutoPathsAzul();
    }
    @Override
    public void onWaitForStart() {
//        angleLL = LimelightHelper.updateAngleLL(limelight);
        Pose poseAtual = PedroComponent.follower().poseTracker.getPose();
        Turret.INSTANCE.setPoseTracker(poseAtual.getX(), poseAtual.getY(), Math.toDegrees(poseAtual.getHeading()), 0, true, telemetry, 0);
        double distanceToGoal = PedroComponent.follower().getPose().distanceFrom(goalPoseazul);
        Shooter.INSTANCE.setGoalDistance(0);
//        TelemetryHelper.addCommonTelemetry(telemetry, PedroComponent.follower().getPose(),
//                Shooter.INSTANCE.getVelocity(), turretAngle, Turret.destinationAngle, toTurn,
//                limelight, angleLL, debugMode, distanceToGoal, 0.0, 0.0, 0.0);
        telemetry.update();
    }
    @Override
    public void onStartButtonPressed() {
//        motor.brakeMode();
        CommandManager.INSTANCE.scheduleCommand(
                new SequentialGroup(
                        preload(),
                        intakeMeio(),
                        shootMeio(),
                        gateCicle(),
                        gateCicle(),
                        intakeCima(),
                        shootCima(),
                        //intakeBaixo(),
                        gateCicle(),
                        //shootfinal(),
                       //gateCicle(),
                        new FollowPath(AutoPathsAzul.last(PedroComponent.follower()))
                )
        );
        PedroComponent.follower().update();
    }
    @Override
    public void onUpdate() {
//
//
//        ControlSystem controlSystem = ControlSystem.builder()
//                .velPid(Fkp, Fki, Fkd)
//                .basicFF(Fkv, Fka, Fks)
//                .build();
//        controlSystem.setGoal(new KineticState(0, goal));
//        double power = controlSystem.calculate(new KineticState(
//                Flywheel.getCurrentPosition(),
//                Flywheel.getVelocity()));
//        Flywheel.setPower(power);
//        LLResult result = limelight.getLatestResult();
//        if (result != null){
//            angleLL = 0;
//        }else{
//            angleLL = 0.0;
//        }
        //Pose poseAtual = PedroComponent.follower().poseTracker.getPose();
        PedroComponent.follower().update();
        Pose poseAtual = PedroComponent.follower().getPose();
        distanceToGoal = PedroComponent.follower().getPose().distanceFrom(goalPoseazul);
        Shooter.INSTANCE.setGoalDistance(distanceToGoal);
        Shooter.INSTANCE.periodic();
        Hood.INSTANCE.setGoalDistance(distanceToGoal);
        Hood.INSTANCE.periodic();
        double lateralVel = PedroComponent.follower().poseTracker.getVelocity().getYComponent();
        double flightTime = distanceToGoal / projectileSpeed;
        double leadCompensation = lateralVel * flightTime;
        Turret.INSTANCE.setPoseTracker(poseAtual.getX(), poseAtual.getY(), poseAtual.getHeading(), 0, true, telemetry, leadCompensation);
//        TelemetryHelper.addCommonTelemetry(telemetry, PedroComponent.follower().getPose(),
//                Shooter.INSTANCE.getVelocity(), turretAngle, Turret.destinationAngle, toTurn,
//                limelight, angleLL, debugMode, distanceToGoal, 0.0, 0.0, 0.0);
        telemetry.update();
    }
    @Override
    public void onStop() {
        //Shooter.INSTANCE.setGoalDistance(0);
        PoseManager.currentPose = PedroComponent.follower().getPose();
    }
    private SequentialGroup shootar() {
        return new SequentialGroup(
                Lock.INSTANCE.open,
                Indexer.INSTANCE.shooting,
                new Delay(0.05),
                Intake.INSTANCE.intake,
                new Delay(0.30),
                Indexer.INSTANCE.naoshooting,
                Lock.INSTANCE.closed
        );
    }
    private SequentialGroup shootpathdist(PathChain path) {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(path),
                        new SequentialGroup(
                                new WaitUntil(() ->
                                        PedroComponent.follower()
                                                .getCurrentPath()
                                                .getClosestPointTValue() > 0.80
                                ),
                                shootar()
                        )
                )
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
                                AutoPathsAzul.Gate(PedroComponent.follower()),
                                true,
                                0.8
                        )
                ),
                new Delay(gatedelay),
                intake(),
                shootpathdist(
                        AutoPathsAzul.ShootGate(PedroComponent.follower())
                )
        );
    }

    private SequentialGroup shootMeio() {
        return shootpathdist(
                AutoPathsAzul.ShootMeio(PedroComponent.follower())
        );
    }
    private SequentialGroup intakeMeio() {
        return new SequentialGroup(
                intake(),
                new FollowPath(AutoPathsAzul.IntakeMeio(PedroComponent.follower())),   //Vai pra fileira do meio
                intake()
        );
    }
    private SequentialGroup preload() {
        return new SequentialGroup(
                shootpathdist(
                        AutoPathsAzul.ShootPreload(PedroComponent.follower())), // Shoot Preload
                shootar(),
                intake()
        );
    }
    private SequentialGroup intakeCima() {
        return new SequentialGroup(
                new ParallelGroup(
                        intake().afterTime(0.5),
                        new FollowPath(AutoPathsAzul.IntakeCima(PedroComponent.follower()))
                ),
                intake()
        );
    }
    private SequentialGroup shootCima() {
        return new SequentialGroup(
                intake(),
                new FollowPath(AutoPathsAzul.ShootCima(PedroComponent.follower())),
                Lock.INSTANCE.open,
                shootar()
        );
    }
    private SequentialGroup intakeBaixo() {
        return new SequentialGroup(
                new ParallelGroup(
                        intake().afterTime(0.8),
                        new FollowPath(AutoPathsAzul.IntakeBaixo(PedroComponent.follower()))
                ),
                intake()
        );
    }
    private SequentialGroup shootfinal() {
        return shootpathdist(
                AutoPathsAzul.shootPoselast(PedroComponent.follower())
        );
    }
//    public class LLalign extends Command {
//        private boolean done = false;
//        private ControlSystem controller;
//        private double tolerancia = tol;
//        private double offset = 0;
//        private double filteredAngle = filter;
//        private final double alpha = carolina;
//        private boolean started = false;
//
//        public LLalign() {
//            requires();
//            setInterruptible(true);
//        }
//        public double getFilteredAngle() {
//            return filteredAngle;
//        }
//
//        @Override
//        public boolean isDone() {
//            return done;
//        }
//
//        @Override
//        public void start() {
//            done = false;
//            controller = ControlSystem.builder()
//                    .posPid(0.008, Tki, Tkd)
//                    .build();
//            filteredAngle = 0;
//        }
//
//        @Override
//        public void update() {
////            angleLL = LimelightHelper.updateAngleLL(limelight);
//            if (angleLL != 0.0 && !started) {
//                started = true;
//
//            }
//
//            if (started) {
//                filteredAngle = filteredAngle + alpha * (angleLL - filteredAngle);
//
//                if (Math.abs(motor.getVelocity()) < 20) {
//                    offset += filteredAngle/4;
//                }
//
//                double targetPosition = motor.getCurrentPosition()
//                        + AutoConstants.Calculos.angleToEncoderTicks(filteredAngle + offset);
//
//                controller.setGoal(new KineticState(targetPosition));
//
//                double power = controller.calculate(
//                        new KineticState(motor.getCurrentPosition(), motor.getVelocity())
//                );
//
//                motor.setPower(clamp(power, -0.5, 0.5));
//
//                if (Math.abs(filteredAngle) < tolerancia && Math.abs(motor.getVelocity()) < 40) {
//                    done = true;
//                    offset = 0;
//                    filteredAngle = 0;
//                }
//            }
//        }
//        @Override
//        public void stop(boolean interrupted) {
//            motor.setPower(0);
//            motor.brakeMode();
//        }
//    }
}

package org.firstinspires.ftc.teamcode.Autos;
import static com.pedropathing.math.MathFunctions.clamp;
import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.goalPoseazul;
import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.poseInicial;
import static org.firstinspires.ftc.teamcode.Constants.AutoPosesFar.PoseInicialFar;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.Tkd;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.Tki;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.Tkp;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.controllerauto;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.toTurn;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.turretAngle;
import com.qualcomm.robotcore.hardware.DcMotor;
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
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.sun.tools.javac.util.MandatoryWarningHandler;

import org.firstinspires.ftc.teamcode.Constants.AutoConstants;
import org.firstinspires.ftc.teamcode.Constants.AutoPathsAzul;
import org.firstinspires.ftc.teamcode.Constants.AutoPathsFar;
import org.firstinspires.ftc.teamcode.Constants.PoseManager;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lock;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Constants.LimelightHelper;
import org.firstinspires.ftc.teamcode.Constants.TelemetryHelper;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous
public class AutoFarAzul extends NextFTCOpMode {
    {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Lock.INSTANCE, Hood.INSTANCE),
                new SubsystemComponent(Shooter.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }

    public static double Fkp = 0.005;
    public static double Fki = 0.000000001;
    public static double Fkd = 0;
    public static double Fks = 0.17;
    public static double Fka = 6;
    public static double Fkv = 0.00028;
    private static final MotorEx motor = new MotorEx("turret").brakeMode();

    private final MotorGroup Flywheel = new MotorGroup(
            new MotorEx("f1"),
            new MotorEx("f2")
    );


    public static boolean debugMode = true;
    public static double distanceToGoal;
    public static double kp = 0.12;
    public static double goal = 1900;
    public static double tol = 1.5;

    private ControlSystem turretHoldController;
    private double turretInitialPosition = 0;

    public static double gatedelay = 1.6;

    public static double carolina = 0.8;
    public static double filter = 0;
    private static ControlSystem controller;
    Limelight3A limelight;
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
        Shooter.INSTANCE.getPower();
        angleLL = 0;
        PedroComponent.follower().setStartingPose(PoseInicialFar);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(400);
        limelight.pipelineSwitch(5);
        limelight.start();
        CommandManager.INSTANCE.scheduleCommand(Lock.INSTANCE.open);
        new AutoPathsAzul();
    }

    @Override
    public void onWaitForStart() {
        angleLL = LimelightHelper.updateAngleLL(limelight);
        Pose poseAtual = PedroComponent.follower().poseTracker.getPose();
        Turret.INSTANCE.setPoseTracker(poseAtual.getX(), poseAtual.getY(), Math.toDegrees(poseAtual.getHeading()), angleLL, true, telemetry, 0);
        double distanceToGoal = PedroComponent.follower().getPose().distanceFrom(goalPoseazul);
        //Shooter.INSTANCE.setGoalDistance(distanceToGoal);

        TelemetryHelper.addCommonTelemetry(telemetry, PedroComponent.follower().getPose(),
                Shooter.INSTANCE.getVelocity(), turretAngle, Turret.destinationAngle, toTurn,
                limelight, angleLL, debugMode, distanceToGoal, 0.0, 0.0, 0.0);
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {

        turretInitialPosition = motor.getCurrentPosition();

        turretHoldController = ControlSystem.builder()
                .posPid(0.008, 0, 0)
                .build();
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Hood.INSTANCE.setHoodPos(Hood.pos);

        CommandManager.INSTANCE.scheduleCommand(
                new SequentialGroup(
                        new Delay(2),
                        new FollowPath(AutoPathsFar.ShootPoseFar1(PedroComponent.follower())),
                        intakePaths(),
                        fileiraPaths(),
                        FinalPaths()
                )
        );

        PedroComponent.follower().update();
    }

    @Override
    public void onUpdate() {

        ControlSystem controlSystem = ControlSystem.builder()
                .velPid(Fkp, Fki, Fkd)
                .basicFF(Fkv, Fka, Fks)
                .build();

        controlSystem.setGoal(new KineticState(0, goal));

        double flywheelPower = controlSystem.calculate(
                new KineticState(
                        Flywheel.getCurrentPosition(),
                        Flywheel.getVelocity()
                )
        );

        Flywheel.setPower(flywheelPower);
        turretHoldController.setGoal(new KineticState(turretInitialPosition));
        double turretPower = turretHoldController.calculate(
                new KineticState(
                        motor.getCurrentPosition(),
                        motor.getVelocity()
                )
        );
        motor.setPower(clamp(turretPower, -0.5, 0.5));

        PedroComponent.follower().update();

        distanceToGoal = PedroComponent.follower()
                .getPose()
                .distanceFrom(goalPoseazul);

        TelemetryHelper.addCommonTelemetry(
                telemetry,
                PedroComponent.follower().getPose(),
                Shooter.INSTANCE.getVelocity(),
                turretAngle,
                Turret.destinationAngle,
                toTurn,
                limelight,
                0,
                debugMode,
                distanceToGoal,
                0.0,
                0.0,
                0.0
        );

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
                Intake.INSTANCE.intake,
                new Delay(0.7),
                Intake.INSTANCE.stop,
                new Delay(0.1),
                Intake.INSTANCE.intake,
                new Delay(0.3)
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
    private SequentialGroup intakePaths() {
        return new SequentialGroup(
                new Delay(gatedelay),
                shootar(),
                intake(),
                new FollowPath(AutoPathsFar.Intake(PedroComponent.follower())),
                new FollowPath(AutoPathsFar.IntakeDuplicate(PedroComponent.follower())),
                //new FollowPath(AutoPathsFar.IntakeDuplicate2(PedroComponent.follower())),
                new FollowPath(AutoPathsFar.ShootPoseFar(PedroComponent.follower())),
                shootar(),
                intake()
        );
    }

    private SequentialGroup fileiraPaths() {
        return new SequentialGroup(
                new FollowPath(AutoPathsFar.FileiraBaixo(PedroComponent.follower())),
                new FollowPath(AutoPathsFar.ShootPoseFar1(PedroComponent.follower())),
                shootar(),
                intake()
        );
    }

    private SequentialGroup FinalPaths() {
        return new SequentialGroup(
                new FollowPath(AutoPathsFar.Line7(PedroComponent.follower())),
                //new FollowPath(AutoPathsFar.Line8(PedroComponent.follower())),
               // new FollowPath(AutoPathsFar.Line9(PedroComponent.follower())),
                new FollowPath(AutoPathsFar.ShootPoseFar(PedroComponent.follower())),
                shootar()
        );
    }

    public class LLalign extends Command {
        private boolean done = false;
        private ControlSystem controller;
        private double tolerancia = tol;
        private double offset = 0;
        private double filteredAngle = filter;
        private final double alpha = carolina;
        private boolean started = false;

        public LLalign() {
            requires();
            setInterruptible(true);
        }
        public double getFilteredAngle() {
            return filteredAngle;
        }
        @Override
        public boolean isDone() {
            return done;
        }

        @Override
        public void start() {
            done = false;
            controller = ControlSystem.builder()
                    .posPid(0.008, Tki, Tkd)
                    .build();
            filteredAngle = 0;
        }
    }
}

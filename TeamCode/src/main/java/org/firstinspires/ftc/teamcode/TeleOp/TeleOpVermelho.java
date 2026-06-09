//package org.firstinspires.ftc.teamcode.TeleOp;
//
//import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.goalPoseazul;
//import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.goalShootPoseAzul;
//import static org.firstinspires.ftc.teamcode.Constants.AutoPoses.poseInicial;
//import static org.firstinspires.ftc.teamcode.Subsystems.Turret.calculatedDestinationAngle;
//import static org.firstinspires.ftc.teamcode.Subsystems.Turret.destinationAngle;
//import static org.firstinspires.ftc.teamcode.Subsystems.Turret.toTurn;
//import static org.firstinspires.ftc.teamcode.Subsystems.Turret.turretAngle;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.field.FieldManager;
//import com.bylazar.field.PanelsField;
//import com.bylazar.field.Style;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.math.Vector;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.teamcode.Constants.PoseManager;
//import org.firstinspires.ftc.teamcode.Constants.TelemetryHelper;
//import org.firstinspires.ftc.teamcode.Subsystems.Hood;
//import org.firstinspires.ftc.teamcode.Subsystems.Intake;
//import org.firstinspires.ftc.teamcode.Subsystems.Lock;
//import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
//import org.firstinspires.ftc.teamcode.Subsystems.Turret;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//import dev.nextftc.core.commands.delays.Delay;
//import dev.nextftc.core.commands.groups.SequentialGroup;
//import dev.nextftc.core.components.BindingsComponent;
//import dev.nextftc.core.components.SubsystemComponent;
//import dev.nextftc.extensions.pedro.PedroComponent;
//import dev.nextftc.ftc.Gamepads;
//import dev.nextftc.ftc.NextFTCOpMode;
//import dev.nextftc.ftc.components.BulkReadComponent;
//
//@Configurable
//@TeleOp(name = "TeleOpVermelho")
//public class TeleOpVermelho extends NextFTCOpMode {
//
//    public TeleOpVermelho() {
//        addComponents(
//                new SubsystemComponent(Turret.INSTANCE),
//                new SubsystemComponent(Intake.INSTANCE),
//                new SubsystemComponent(Lock.INSTANCE),
//                new SubsystemComponent(Hood.INSTANCE),
//                new PedroComponent(Constants::createFollower),
//                BulkReadComponent.INSTANCE,
//                BindingsComponent.INSTANCE
//        );
//    }
//
//    private static final Style robotLook = new Style("", "#3F51B5", 0.75);
//    private static final Style robotLook2 = new Style("", "#800000", 0.75);
//    private static final Style robotLook3 = new Style("", "#008000", 0.75);
//
//    private static final FieldManager panelsField =
//            PanelsField.INSTANCE.getField();
//
//    private DcMotor FrontLeft;
//    private DcMotor FrontRight;
//    private DcMotor BackLeft;
//    private DcMotor BackRight;
//
//    private double compensationX = 0.0;
//    private double compensationY = 0.0;
//
//    public static double compensation = 1.45;
//    public static boolean debugMode = true;
//
//    @Override
//    public void onInit() {
//
//        FrontLeft = hardwareMap.get(DcMotor.class, "fl");
//        FrontRight = hardwareMap.get(DcMotor.class, "fr");
//        BackLeft = hardwareMap.get(DcMotor.class, "bl");
//        BackRight = hardwareMap.get(DcMotor.class, "br");
//
//        BackLeft.setDirection(DcMotor.Direction.REVERSE);
//        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
//
//        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        panelsField.setOffsets(
//                PanelsField.INSTANCE
//                        .getPresets()
//                        .getPEDRO_PATHING()
//        );
//
//        Lock.INSTANCE.closed.invoke();
//
//        Pose startPose =
//                (PoseManager.currentPose != null)
//                        ? PoseManager.currentPose
//                        : poseInicial;
//
//        PedroComponent.follower().setStartingPose(startPose.mirror());
//    }
//
//    @Override
//    public void onStartButtonPressed() {
//
//        Pose poseAtual = PedroComponent.follower().getPose();
//
//        Turret.INSTANCE.setPoseTracker(
//                poseAtual.getX(),
//                poseAtual.getY(),
//                poseAtual.getHeading(),
//                0.0,
//                false,
//                telemetry,
//                0.0
//
//        );
//
//        Gamepads.gamepad1().leftBumper().whenBecomesTrue(() ->
//                Lock.INSTANCE.closed.schedule()
//        );
//
//        Gamepads.gamepad1().leftBumper().whenTrue(() ->
//                Intake.INSTANCE.intake.schedule()
//        );
//
//        Gamepads.gamepad1().leftBumper().whenBecomesFalse(() ->
//                Intake.INSTANCE.stop.schedule()
//        );
//
//        Gamepads.gamepad1().y().whenTrue(() ->
//                Intake.INSTANCE.reversed.schedule()
//        );
//
//        Gamepads.gamepad1().dpadRight().whenBecomesTrue(
//                Turret::addRecOffset
//        );
//
//        Gamepads.gamepad1().b().whenBecomesTrue(
//                Turret::lessRecOffset
//        );
//
//        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() ->
//                new SequentialGroup(
//                        Lock.INSTANCE.open,
//                        Intake.INSTANCE.shooting,
//                        new Delay(0.7),
//                        Intake.INSTANCE.stop
//                ).schedule()
//        );
//
//        Gamepads.gamepad1().x().whenBecomesTrue(
//                this::resetPose
//        );
//
//        Gamepads.gamepad1().a().whenBecomesTrue(
//                this::resetTurretPose
//        );
//    }
//
//    @Override
//    public void onUpdate() {
//
//        PedroComponent.follower().update();
//        Pose poseAtual = PedroComponent.follower().getPose();
//
//        Turret.INSTANCE.setPoseTracker(
//                poseAtual.getX(),
//                poseAtual.getY(),
//                poseAtual.getHeading(),
//                0.0,
//                false,
//                telemetry,
//                0.0
//
//        );
//
//        double y = -gamepad1.left_stick_y;
//        double x = gamepad1.left_stick_x;
//        double rx = gamepad1.right_stick_x;
//
//        double headingRad =
//                PedroComponent.follower()
//                        .getPose()
//                        .getHeading();
//
//        double rotX =
//                x * Math.cos(-headingRad)
//                        - y * Math.sin(-headingRad);
//
//        double rotY =
//                x * Math.sin(-headingRad)
//                        + y * Math.cos(-headingRad);
//
//        rotX *= 1.1;
//
//        double denominator = Math.max(
//                Math.abs(rotY)
//                        + Math.abs(rotX)
//                        + Math.abs(rx),
//                1.0
//        );
//
//        FrontLeft.setPower((rotY + rotX + rx) / denominator);
//        BackLeft.setPower((rotY - rotX + rx) / denominator);
//        FrontRight.setPower((rotY - rotX - rx) / denominator);
//        BackRight.setPower((rotY + rotX - rx) / denominator);
//
//
//        double distanceToGoal =
//                poseAtual.distanceFrom(goalShootPoseAzul);
//
//        Shooter.INSTANCE.setGoalDistance(distanceToGoal);
//        Shooter.INSTANCE.periodic();
//
//        Hood.INSTANCE.setGoalDistance(distanceToGoal);
//        Hood.INSTANCE.periodic();
//
//        Vector v = new Pose(
//                0,
//                0,
//                Math.toRadians(calculatedDestinationAngle)
//        ).getHeadingAsUnitVector();
//
//        v.setMagnitude(v.getMagnitude() * 9);
//
//        Vector v2 = new Pose(
//                0,
//                0,
//                Math.toRadians(destinationAngle)
//        ).getHeadingAsUnitVector();
//
//        v2.setMagnitude(v2.getMagnitude() * 9);
//
//        Vector v3 = new Pose(
//                0,
//                0,
//                Math.toRadians(toTurn)
//        ).getHeadingAsUnitVector();
//
//        v3.setMagnitude(v3.getMagnitude() * 9);
//
//        panelsField.setStyle(robotLook);
//        panelsField.moveCursor(poseAtual.getX(), poseAtual.getY());
//        panelsField.circle(9);
//
//        panelsField.setStyle(robotLook);
//        panelsField.moveCursor(
//                poseAtual.getX() + v.getXComponent() / 2,
//                poseAtual.getY() + v.getYComponent() / 2
//        );
//        panelsField.line(
//                poseAtual.getX() + v.getXComponent(),
//                poseAtual.getY() + v.getYComponent()
//        );
//
//        panelsField.setStyle(robotLook2);
//        panelsField.moveCursor(
//                poseAtual.getX() + v2.getXComponent() / 2,
//                poseAtual.getY() + v2.getYComponent() / 2
//        );
//        panelsField.line(
//                poseAtual.getX() + v2.getXComponent(),
//                poseAtual.getY() + v2.getYComponent()
//        );
//
//        panelsField.setStyle(robotLook3);
//        panelsField.moveCursor(
//                poseAtual.getX() + v3.getXComponent() / 2,
//                poseAtual.getY() + v3.getYComponent()
//        );
//        panelsField.line(
//                poseAtual.getX() + v3.getXComponent(),
//                poseAtual.getY() + v3.getYComponent()
//        );
//
//        panelsField.update();
//
//        TelemetryHelper.addCommonTelemetry(
//                telemetry,
//                PedroComponent.follower().getPose(),
//                Shooter.INSTANCE.getVelocity(),
//                turretAngle,
//                Turret.destinationAngle,
//                toTurn,
//                null,
//                0.0,
//                debugMode,
//                distanceToGoal,
//                0.0,
//                0.0,
//                0.0
//        );
//
//        telemetry.update();
//    }
//
//    private void resetPose() {
//        PedroComponent.follower().setPose(poseInicial);
//    }
//
//    private void resetTurretPose() {
//        Turret.INSTANCE.resetTurret();
//    }
//}
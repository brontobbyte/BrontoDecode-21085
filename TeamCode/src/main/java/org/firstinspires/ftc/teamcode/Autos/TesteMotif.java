package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Config
public class TesteMotif extends NextFTCOpMode {
    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Shooter.INSTANCE)
        );
    }
    public static double PoseInicialX = 27.468208092485547;
    public static double PoseInicialY = 128;
    public static double ShootInicialPoseX = 43.616;
    public static double ShootInicialPoseY = 117.864;
    public static double GoIntakeCurvedPoseX = 73.121;
    public static double GoIntakeCurvedPoseY = 31.402;
    public static double GoIntakeX = 41.495;
    public static double GoIntakeY = 34.766;
    public static double Intake2PoseX = 18.841;
    public static double Intake2PoseY = 34.991;
    public static double Shoot2PoseX = 71.327;
    public static double Shoot2PoseY = 77.383;
    public static double Shoot2CurvedPoseX = 75.364;
    public static double Shoot2CurvedPoseY = 35.888;
    public static double GoPoseX = 35.888;
    public static double GoPoseY = 71.77570093457945;
    public static double GoIntake3PoseX = 35.88785046728972;
    public static double GoIntake3PoseY = 59.214953271028044;
    public static double Intake3PoseX = 18.8411214953271;
    public static double Intake3PoseY = 59.214953271028044;
    public static double Shoot3PoseX = 40.37383177570093;
    public static double Shoot3PoseY = 116.41121495327101;






















    public static Pose PoseInicial = new Pose(PoseInicialX, PoseInicialY);
    public static Pose ShootInicialPose = new Pose(ShootInicialPoseX, ShootInicialPoseY);
    public static Pose GoIntakeCurvedPose = new Pose(GoIntakeCurvedPoseX, GoIntakeCurvedPoseY);
    public static Pose GoIntakePose = new Pose(GoIntakeX, GoIntakeY );
    public static Pose Intake2Pose = new Pose(Intake2PoseX, Intake2PoseY);
    public static Pose Shoot2Pose = new Pose(Shoot2PoseX, Shoot2PoseY);
    public static Pose Shoot2CurvedPose = new Pose(Shoot2CurvedPoseX, Shoot2CurvedPoseY );
    public static Pose GoPose = new Pose(GoPoseX, GoPoseY);
    public static Pose GoIntake3Pose = new Pose(GoIntake3PoseX, GoIntake3PoseY);
    public static Pose Intake3Pose = new Pose(Intake3PoseX, Intake3PoseY);
    public static Pose Shoot3Pose = new Pose(Shoot3PoseX, Shoot3PoseY);
    private final Pose startPose = new Pose(72, 120, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(72, 20, Math.toRadians(115)); // Scoring Pose of our robot. It is facing the goal at a 115 degree angle.
    private final Pose PPGPose = new Pose(100, 83.5, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose PGPPose = new Pose(100, 59.5, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose GPPPose = new Pose(100, 35.5, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private Path scorePreload;
    private PathChain ShootInicial, GoIntake2, Intake2, Shoot2, Go, GoIntake3, Intake3, Shoot3, Park;
    private PathChain grabPPG, scorePPG, grabPGP, scorePGP, grabGPP, scoreGPP;
    private final ElapsedTime runtime = new ElapsedTime();


    private static final int PPG_TAG_ID = 23;
    private static final int PGP_TAG_ID = 22;

    private static final int GPP_TAG_ID = 21;
    private static final boolean USE_WEBCAM = true;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;
    // Other variables
    private Pose currentPose;
    private Follower follower;
    private TelemetryManager panelsTelemetry;
    private int pathStatePPG;
    private int pathStatePGP;
    private int pathStateGPP;
    private int foundID;

    private void log(String caption, Object... text) {
        if (text.length == 1) {
            telemetry.addData(caption, text[0]);
            panelsTelemetry.debug(caption + ": " + text[0]);
        } else if (text.length >= 2) {
            StringBuilder message = new StringBuilder();
            for (int i = 0; i < text.length; i++) {
                message.append(text[i]);
                if (i < text.length - 1) message.append(" ");
            }
            telemetry.addData(caption, message.toString());
            panelsTelemetry.debug(caption + ": " + message);
        }
    }


    public void buildPaths() {


        ShootInicial = follower.pathBuilder()
                .addPath(
                        new BezierLine(PoseInicial, ShootInicialPose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(-36), Math.toRadians(-36))
                .build();

        GoIntake2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                ShootInicialPose,
                                GoIntakeCurvedPose,
                                GoIntakePose
                        )
                )

                .setLinearHeadingInterpolation(Math.toRadians(-36), Math.toRadians(180))
                .build();

        Intake2 = follower.pathBuilder()
                .addPath(new BezierLine(GoIntakePose, Intake2Pose))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(Intake2Pose, Shoot2Pose))
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-90))
                .build();

        Go = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                Shoot2Pose,
                                Shoot2CurvedPose,
                                GoPose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-180))
                .build();

        GoIntake3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                GoPose,
                                GoIntake3Pose
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .build();

        Intake3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                GoIntake3Pose,
                                Intake3Pose
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .build();

        Shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(Intake3Pose, Shoot3Pose))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .build();
    }


    @Override public void onInit() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        initAprilTag();
        
        if (USE_WEBCAM) {
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        }
        // Log completed initialization to Panels and driver station (custom log function)
        log("Status", "Initialized");
        telemetry.update(); // Update driver station after logging

        //Wait For Start vai daqui
        waitForStart();
        runtime.reset();

        setpathStatePPG(0);
        setpathStatePGP(0);
        setpathStateGPP(0);
        runtime.reset();
// até aqui
        while (opModeIsActive()) {

        }
    }


    private void setpathStatePPG(int i) {
    }
    private void setpathStatePGP(int i) {}
    private void setpathStateGPP(int i) {}


    private void setManualExposure(int i, int i1) {
    }

    private void initAprilTag() {
    }


    @Override public void onWaitForStart() {

    }
    @Override public void onStartButtonPressed() {
        new SequentialGroup(
                new FollowPath(ShootInicial)
        );
    }

    @Override public void onUpdate() {

        // Update Pedro Pathing and Panels every iteration
        follower.update();
        panelsTelemetry.update();
        currentPose = follower.getPose(); // Update the current pose
        boolean targetFound = false;
        desiredTag = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections){

            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if (detection.id == PPG_TAG_ID) {

                    // call lines for the PGP pattern
                    buildPathsPPG();
                    targetFound = true;
                    desiredTag = detection;
                    foundID = 21; // This should likely be PPG_TAG_ID or the corresponding state machine ID
                    break;  // don't look any further.
                } else if (detection.id == PGP_TAG_ID) {
                // call lines for the PGP pattern
                buildPathsPGP();
                targetFound = true;
                desiredTag = detection;
                foundID = 22; // This should likely be PGP_TAG_ID or the corresponding state machine ID
                break;  // don't look any further.
            } else if (detection.id == GPP_TAG_ID) {
                // call lines for the GPP pattern
                buildPathsGPP();
                targetFound = true;
                desiredTag = detection;
                foundID = 23; // This should likely be GPP_TAG_ID or the corresponding state machine ID
                break;  // don't look any further.
            }
        } else {
            // This tag is NOT in the library, so we don't have enough information to track to it.
            telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
        // Update the state machine
        if (foundID == 21) { // Consider using the TAG_ID constants or a dedicated variable for which path was found
            updateStateMachinePPG();
        } else if (foundID == 22) {
            updateStateMachinePGP();
        } else if (foundID == 23) {
            updateStateMachineGPP();
        }
        // Log to Panels and driver station (custom log function)
        log("Elapsed", runtime.toString());
        log("X", currentPose.getX());
        log("Y", currentPose.getY());
        log("Heading", currentPose.getHeading());
        telemetry.update();
    }



    private void updateStateMachinePPG() {
    }
    private void updateStateMachinePGP(){
    }

    private void updateStateMachineGPP(){
    }


    private void buildPathsPGP() {
        new Auto_PGP.PGPPaths();
    }

    private void buildPathsPPG() {
        new Auto_PPG.PPGPaths();
    }

    private void buildPathsGPP () {
        new Auto_GPP.GPPPaths();
    }

}



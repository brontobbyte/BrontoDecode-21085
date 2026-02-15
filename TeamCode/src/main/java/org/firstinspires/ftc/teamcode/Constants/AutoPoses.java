package org.firstinspires.ftc.teamcode.Constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
@Configurable
public class AutoPoses {
    public static Pose poseInicial       = new Pose(108, 108, Math.toRadians(90));
    public static Pose PoseInicialfar    = new Pose(56, 8, -Math.toRadians(180));
    public static Pose intakeCurvedPose  = new Pose(94, 50, Math.toRadians(0));
    public static Pose intakePose        = new Pose(36, 106, Math.toRadians(0));
    public static Pose shootPose1        = new Pose(56.726, 75.225, Math.toRadians(0));
    public static Pose intakeGatePose    = new Pose(11.820, 57.5, Math.toRadians(0));
    public static Pose shootPose2        = new Pose(56.837, 75.410, Math.toRadians(0));
    public static Pose intake2CurvedPose = new Pose(43.463, 86.782, Math.toRadians(0));
    public static Pose intake2Pose       = new Pose(18.096, 80, Math.toRadians(0));
    public static Pose intake3CurvedPose = new Pose(88.015, 30.053, Math.toRadians(0));
    public static Pose intake3Pose       = new Pose(17.651, 54, Math.toRadians(0));
    public static Pose goalPose          = new Pose(10, 137, Math.toRadians(0));

}


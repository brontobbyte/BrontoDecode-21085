package org.firstinspires.ftc.teamcode.Constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
@Configurable
public class AutoPoses {
    public static Pose poseInicial       = new Pose(19.209, 121.202, Math.toRadians(90));
    public static Pose intakeCurvedPose  = new Pose(87.102, 58.520, Math.toRadians(0));
    public static Pose intakePose        = new Pose(19.644, 59.764, Math.toRadians(0));
    public static Pose shootPose1        = new Pose(56.726, 75.225, Math.toRadians(0));
    public static Pose intakeGatePose    = new Pose(11.820, 59.598, Math.toRadians(0));
    public static Pose shootPose2        = new Pose(56.837, 75.410, Math.toRadians(0));
    public static Pose intake2CurvedPose = new Pose(43.463, 86.782, Math.toRadians(0));
    public static Pose intake2Pose       = new Pose(18.096, 83.838, Math.toRadians(0));
    public static Pose intake3CurvedPose = new Pose(88.015, 30.053, Math.toRadians(0));
    public static Pose intake3Pose       = new Pose(17.651, 35.303, Math.toRadians(0));
}


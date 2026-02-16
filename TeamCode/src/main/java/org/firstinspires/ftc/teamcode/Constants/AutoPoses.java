package org.firstinspires.ftc.teamcode.Constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
@Configurable
public class AutoPoses {
    public static Pose poseInicial          = new Pose(20, 118, Math.toRadians(90));
    public static Pose PoseInicialfar       = new Pose(56, 8, -Math.toRadians(180));
    public static Pose intakeMeioCurvedPose = new Pose(63, 57, Math.toRadians(0));
    public static Pose intakeMeioPose       = new Pose(15, 56, Math.toRadians(0));
    public static Pose gatePose             = new Pose(14, 64, Math.toRadians(0));
    public static Pose gateCiclePose        = new Pose(10.95412844036698, 53.47706422018349, Math.toRadians(0));
    public static Pose gateCicleCurvedPose  = new Pose(28, 56, Math.toRadians(0));
    public static Pose shootPose1           = new Pose(62, 68, Math.toRadians(0));
    public static Pose shootPose2           = new Pose(51, 84, Math.toRadians(0));
    public static Pose shootCurvedPose2     = new Pose(41, 55, Math.toRadians(0));
    public static Pose shootPose3           = new Pose(60, 108, Math.toRadians(0));
    public static Pose intakeCimaPose       = new Pose(20, 88, Math.toRadians(0));
    public static Pose goalPose             = new Pose(10, 137, Math.toRadians(0));

}


package org.firstinspires.ftc.teamcode.Constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
@Configurable
public class AutoPoses {
    public static Pose poseInicial           = new Pose(20, 118, Math.toRadians(90));
    public static Pose PoseInicialfar        = new Pose(56, 8, -Math.toRadians(180));
    public static Pose intakeMeioCurvedPose  = new Pose(63, 57, Math.toRadians(0));
    public static Pose intakeMeioPose        = new Pose(15, 53, Math.toRadians(0));
    public static Pose gatePose              = new Pose(16, 60, Math.toRadians(0));
    public static Pose gateCiclePose         = new Pose(8.95412844036698, 53.47706422018349, Math.toRadians(0));
    public static Pose gateCicleCurvedPose   = new Pose(9, 60, Math.toRadians(0));
    public static Pose gateCiclePoseFinal    = new Pose(8, 64, Math.toRadians(0));
    public static Pose gateCicleCurvedPoseFinal    = new Pose(6, 53, Math.toRadians(0));

    public static Pose shootPose1            = new Pose(55, 68, Math.toRadians(0));
    public static Pose shootPose2            = new Pose(45, 78, Math.toRadians(0));
    public static Pose shootCurvedPose2      = new Pose(41, 55, Math.toRadians(0));
    public static Pose shootPose3            = new Pose(55, 108, Math.toRadians(0));
    public static Pose intakeCimaPose        = new Pose(15, 78, Math.toRadians(0));
    public static Pose intakeBaixoPose       = new Pose(17, 32, Math.toRadians(0));
    public static Pose intakeBaixoCurvedPose = new Pose(75, 24, Math.toRadians(0));
    public static Pose goalPose              = new Pose(10, 137, Math.toRadians(0));
    public static Pose shootPoselast         = new Pose(60, 80, Math.toRadians(0));
    public static Pose last                  = new Pose(45, 63, Math.toRadians(0));
}


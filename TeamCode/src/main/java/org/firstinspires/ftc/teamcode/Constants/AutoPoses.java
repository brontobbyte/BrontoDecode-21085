package org.firstinspires.ftc.teamcode.Constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
@Configurable
public class AutoPoses {
    public static Pose poseInicial           = new Pose(26.96435342014482, 126.87326351120366, Math.toRadians(180));
    //public static Pose poseResetHumanPAzul       = new Pose(12.142586319420701, 76.6950306119278, Math.toRadians(180));
    public static Pose poseResetHumanPAzul       = new Pose(17.53115264797508, 78.58333333333333, Math.toRadians(180));
    public static Pose poseResetHumanPVemrelho    = new Pose(17.53115264797508, 78.58333333333333, Math.toRadians(180)).mirror();
    public static Pose intakeMeioCurvedPose  = new Pose(60, 59.49504304316747, Math.toRadians(0));
    public static Pose intakeMeioPose        = new Pose(12, 59.49504304316747, Math.toRadians(0));
    public static Pose gatePose              = new Pose(7.75, 62.5, Math.toRadians(155));
    public static Pose gatePose2             = new Pose(28, 55, Math.toRadians(0));
    public static Pose gateCurvedPose        = new Pose(50, 48, Math.toRadians(70));
    public static Pose gateCiclePose         = new Pose(17.5, 56, Math.toRadians(0));
    public static Pose gateCicleCurvedPose   = new Pose(15, 60, Math.toRadians(0));
    public static Pose gateCiclePoseFinal    = new Pose(8, 64, Math.toRadians(0));
    public static Pose shootPose1            = new Pose(56, 77, Math.toRadians(180));
    public static Pose shootPose2            = new Pose(45, 78, Math.toRadians(0));
    public static Pose shootCurvedPose2      = new Pose(41, 55, Math.toRadians(0));
    public static Pose shootPose3            = new Pose(55, 108, Math.toRadians(0));
    public static Pose intakeCimaPose        = new Pose(20, 84, Math.toRadians(0));
    public static Pose intakeCimaCurvedPose  = new Pose(55, 90, Math.toRadians(0));
    public static Pose intakeBaixoPose       = new Pose(15, 38, Math.toRadians(0));
    public static Pose intakeBaixoCurvedPose = new Pose(70, 35, Math.toRadians(0));
    public static Pose goalPoseazul          = new Pose(0, 144, Math.toRadians(0));
    public static Pose goalPoseVermelho      = new Pose(144, 144, Math.toRadians(0));
    public static Pose goalShootPoseVermelho      = new Pose(144, 144, Math.toRadians(0));
    public static Pose goalShootPoseAzul      = new Pose(0, 144, Math.toRadians(0));
    public static Pose shootPoselast         = new Pose(60, 80, Math.toRadians(0));
    public static Pose last                  = new Pose(45, 63, Math.toRadians(0));
}


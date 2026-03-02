//
//
///* ============================================================= *
// *        Pedro Pathing Plus Visualizer — Auto-Generated         *
// *                                                               *
// *  Version: 1.7.4.                                              *
// *  Copyright (c) 2026 Matthew Allen                             *
// *                                                               *
// *  THIS FILE IS AUTO-GENERATED — DO NOT EDIT MANUALLY.          *
// *  Changes will be overwritten when regenerated.                *
// * ============================================================= */
//
//
//package org.firstinspires.ftc.teamcode.Autos;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import dev.nextftc.core.commands.groups.SequentialGroup;
//import dev.nextftc.core.commands.groups.ParallelRaceGroup;
//import dev.nextftc.core.commands.Delay;
//import dev.nextftc.core.commands.WaitUntil;
//import dev.nextftc.core.command.InstantCommand;
//import dev.nextftc.extensions.pedro.command.FollowPath;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//import com.pedropathingplus.pathing.ProgressTracker;
//import com.pedropathingplus.pathing.NamedCommands;
//import java.io.IOException;
//import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
//
//public class trajectory__4_ extends SequentialGroup {
//
//    private final Follower follower;
//    private final ProgressTracker progressTracker;
//
//    // Poses
//    private Pose startPoint;
//    private Pose PreLoadEFlileiraMeio;
//    private Pose PreLoadEFlileiraMeio_line0_control1;
//    private Pose ShootPose;
//    private Pose PrimeiraFileira;
//    private Pose ShootGate;
//    private Pose Gate;
//    private Pose Gate_line4_control1;
//    private Pose GiradinhaGate;
//    private Pose Gate_line7_control1;
//    private Pose 3fileira;
//    private Pose 3fileira_line10_control1;
//    private Pose point12;
//
//    // Path chains
//    private PathChain startPointTOPreLoadEFlileiraMeio;
//    private PathChain PreLoadEFlileiraMeioTOShootPose;
//    private PathChain ShootPoseTOPrimeiraFileira;
//    private PathChain PrimeiraFileiraTOShootGate;
//    private PathChain ShootGateTOGate;
//    private PathChain GateTOGiradinhaGate;
//    private PathChain GiradinhaGateTOShootGate;
//    private PathChain ShootGateTOGate_1;
//    private PathChain GateTOGiradinhaGate_1;
//    private PathChain GiradinhaGateTOShootGate_1;
//    private PathChain ShootGateTO3fileira;
//    private PathChain 3fileiraTOpoint12;
//
//    public trajectory__4_(final Drivetrain drive, HardwareMap hw, Telemetry telemetry) throws IOException {
//        this.follower = drive.getFollower();
//        this.progressTracker = new ProgressTracker(follower, telemetry);
//
//
//
//        // Load poses
//        startPoint = new Pose(21.594, 123.069, Math.toRadians(143));
//        PreLoadEFlileiraMeio = new Pose(24.055, 59.495, Math.toRadians(180));
//        PreLoadEFlileiraMeio_line0_control1 = new Pose(69.630, 59.495);
//        ShootPose = new Pose(51.907, 79.756, Math.toRadians(180));
//        PrimeiraFileira = new Pose(24.055, 83.905, Math.toRadians(180));
//        ShootGate = new Pose(51.907, 79.756, Math.toRadians(180));
//        Gate = new Pose(16.000, 64.642, Math.toRadians(180));
//        Gate_line4_control1 = new Pose(43.836, 63.818);
//        GiradinhaGate = new Pose(12.420, 59.495, Math.toRadians(145));
//        Gate_line7_control1 = new Pose(43.836, 63.818);
//        3fileira = new Pose(24.055, 35.648, Math.toRadians(180));
//        3fileira_line10_control1 = new Pose(42.693, 32.766);
//        point12 = new Pose(59.012, 70.616, Math.toRadians(-145));
//
//        follower.setStartingPose(startPoint);
//
//        buildPaths();
//
//        addCommands(
//                new InstantCommand(
//                        () -> {
//                            progressTracker.setCurrentChain(startPointTOPreLoadEFlileiraMeio);
//                            progressTracker.setCurrentPathName("startPointTOPreLoadEFlileiraMeio");
//                        }),
//                new FollowPath(startPointTOPreLoadEFlileiraMeio),
//                new InstantCommand(
//                        () -> {
//                            progressTracker.setCurrentChain(PreLoadEFlileiraMeioTOShootPose);
//                            progressTracker.setCurrentPathName("PreLoadEFlileiraMeioTOShootPose");
//                        }),
//                new FollowPath(PreLoadEFlileiraMeioTOShootPose),
//                new InstantCommand(
//                        () -> {
//                            progressTracker.setCurrentChain(ShootPoseTOPrimeiraFileira);
//                            progressTracker.setCurrentPathName("ShootPoseTOPrimeiraFileira");
//                        }),
//                new FollowPath(ShootPoseTOPrimeiraFileira),
//                new InstantCommand(
//                        () -> {
//                            progressTracker.setCurrentChain(PrimeiraFileiraTOShootGate);
//                            progressTracker.setCurrentPathName("PrimeiraFileiraTOShootGate");
//                        }),
//                new FollowPath(PrimeiraFileiraTOShootGate),
//                new InstantCommand(
//                        () -> {
//                            progressTracker.setCurrentChain(ShootGateTOGate);
//                            progressTracker.setCurrentPathName("ShootGateTOGate");
//                        }),
//                new FollowPath(ShootGateTOGate),
//                new InstantCommand(
//                        () -> {
//                            progressTracker.setCurrentChain(GateTOGiradinhaGate);
//                            progressTracker.setCurrentPathName("GateTOGiradinhaGate");
//                        }),
//                new FollowPath(GateTOGiradinhaGate),
//                new Delay(2.000),
//                new InstantCommand(
//                        () -> {
//                            progressTracker.setCurrentChain(GiradinhaGateTOShootGate);
//                            progressTracker.setCurrentPathName("GiradinhaGateTOShootGate");
//                        }),
//                new FollowPath(GiradinhaGateTOShootGate),
//                new InstantCommand(
//                        () -> {
//                            progressTracker.setCurrentChain(ShootGateTOGate_1);
//                            progressTracker.setCurrentPathName("ShootGateTOGate_1");
//                        }),
//                new FollowPath(ShootGateTOGate_1),
//                new InstantCommand(
//                        () -> {
//                            progressTracker.setCurrentChain(GateTOGiradinhaGate_1);
//                            progressTracker.setCurrentPathName("GateTOGiradinhaGate_1");
//                        }),
//                new FollowPath(GateTOGiradinhaGate_1),
//                new Delay(2.000),
//                new InstantCommand(
//                        () -> {
//                            progressTracker.setCurrentChain(GiradinhaGateTOShootGate_1);
//                            progressTracker.setCurrentPathName("GiradinhaGateTOShootGate_1");
//                        }),
//                new FollowPath(GiradinhaGateTOShootGate_1),
//                new InstantCommand(
//                        () -> {
//                            progressTracker.setCurrentChain(ShootGateTO3fileira);
//                            progressTracker.setCurrentPathName("ShootGateTO3fileira");
//                        }),
//                new FollowPath(ShootGateTO3fileira),
//                new InstantCommand(
//                        () -> {
//                            progressTracker.setCurrentChain(3fileiraTOpoint12);
//                            progressTracker.setCurrentPathName("3fileiraTOpoint12");
//                        }),
//                new FollowPath(3fileiraTOpoint12)
//        );
//    }
//
//    public void buildPaths() {
//        startPointTOPreLoadEFlileiraMeio = follower.pathBuilder()
//                .addPath(new BezierCurve(startPoint, PreLoadEFlileiraMeio_line0_control1, PreLoadEFlileiraMeio))
//                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
//                .build();
//
//        PreLoadEFlileiraMeioTOShootPose = follower.pathBuilder()
//                .addPath(new BezierLine(PreLoadEFlileiraMeio, ShootPose))
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//                .setReversed()
//                .build();
//
//        ShootPoseTOPrimeiraFileira = follower.pathBuilder()
//                .addPath(new BezierLine(ShootPose, PrimeiraFileira))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//        PrimeiraFileiraTOShootGate = follower.pathBuilder()
//                .addPath(new BezierLine(PrimeiraFileira, ShootGate))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .setReversed()
//                .build();
//
//        ShootGateTOGate = follower.pathBuilder()
//                .addPath(new BezierCurve(ShootGate, Gate_line4_control1, Gate))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//        GateTOGiradinhaGate = follower.pathBuilder()
//                .addPath(new BezierLine(Gate, GiradinhaGate))
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
//                .build();
//
//        GiradinhaGateTOShootGate = follower.pathBuilder()
//                .addPath(new BezierLine(GiradinhaGate, ShootGate))
//                .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
//                .setReversed()
//                .build();
//
//        ShootGateTOGate_1 = follower.pathBuilder()
//                .addPath(new BezierCurve(ShootGate, Gate_line7_control1, Gate))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//        GateTOGiradinhaGate_1 = follower.pathBuilder()
//                .addPath(new BezierLine(Gate, GiradinhaGate))
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
//                .build();
//
//        GiradinhaGateTOShootGate_1 = follower.pathBuilder()
//                .addPath(new BezierLine(GiradinhaGate, ShootGate))
//                .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
//                .setReversed()
//                .build();
//
//        ShootGateTO3fileira = follower.pathBuilder()
//                .addPath(new BezierCurve(ShootGate, 3fileira_line10_control1, 3fileira))
//                .setLinearHeadingInterpolation(Math.toRadians(-130), Math.toRadians(180))
//                .build();
//
//        3fileiraTOpoint12 = follower.pathBuilder()
//                .addPath(new BezierLine(3fileira, point12))
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-145))
//                .build();
//    }
//}

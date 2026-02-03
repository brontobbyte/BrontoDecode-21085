
package org.firstinspires.ftc.teamcode.Constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

@Configurable
public class PathsTeleop {
    public PathChain Gate;
    public PathChain Intake;

    public PathsTeleop(Follower follower) {
        Gate = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(29.51, 71.55),
                                new Pose(19.4545, 70.65)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        Intake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(19.4545, 70.65),
                                new Pose(14.98, 39.13),
                                new Pose(5.81, 58.58)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                .build();
    }
}
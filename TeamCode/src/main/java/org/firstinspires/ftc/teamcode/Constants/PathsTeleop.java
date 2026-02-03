
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
                                new Pose(26.936, 65.908),
                                new Pose(14.826, 65.725)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        Intake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(14.826, 65.725),
                                new Pose(10.752, 54.009),
                                new Pose(5.358, 58.862)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                .build();
    }
}
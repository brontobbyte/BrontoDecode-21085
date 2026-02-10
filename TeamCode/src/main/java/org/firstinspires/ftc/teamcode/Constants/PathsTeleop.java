
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
                                new Pose(56, 87),
                                new Pose(48, 95)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-40), Math.toRadians(140))
                .build();
    }
}
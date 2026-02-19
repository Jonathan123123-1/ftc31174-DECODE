package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class AutoTestFull extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private PathChain[] paths;
    private int currentPathIndex = 0;
    private boolean pathStarted = false;

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();

        // Only set the starting pose once
        follower.setPose(new Pose(22.082, 121.918, Math.toRadians(135)));
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        pathTimer.resetTimer();
    }

    private void buildPaths() {
        paths = new PathChain[7];

        // Path 1 - Linear
        paths[0] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(22.082, 121.918), new Pose(71.222, 72.156)))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(138))
                .build();

        // Path 2 - Tangential
        paths[1] = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(71.222, 72.156),
                        new Pose(40.432, 84.596),
                        new Pose(19.283, 84.285)
                ))
                .setTangentHeadingInterpolation()
                .build();

        // Path 3 - Linear
        paths[2] = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(19.283, 84.285),
                        new Pose(41.054, 84.596),
                        new Pose(70.911, 72.156)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(161))
                .build();

        // Path 4 - Tangential
        paths[3] = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(70.911, 72.156),
                        new Pose(42.298, 59.715),
                        new Pose(18.039, 59.715)
                ))
                .setTangentHeadingInterpolation()
                .build();

        // Path 5 - Linear
        paths[4] = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(18.039, 59.715),
                        new Pose(41.987, 60.337),
                        new Pose(70.911, 72.467)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 6 - Tangential
        paths[5] = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(70.911, 72.467),
                        new Pose(37.633, 30.168),
                        new Pose(39.810, 35.767),
                        new Pose(18.039, 35.767)
                ))
                .setTangentHeadingInterpolation()
                .build();

        // Path 7 - Linear
        paths[6] = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(18.039, 35.767), new Pose(71.222, 73.089)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    @Override
    public void loop() {
        follower.update();

        // Start next path only if previous path is finished
        if (currentPathIndex < paths.length) {
            if (!follower.isBusy() && !pathStarted) {
                follower.followPath(paths[currentPathIndex], true);
                pathStarted = true;
                pathTimer.resetTimer();
            }

            // If path finished, move to next path
            if (pathStarted && !follower.isBusy()) {
                currentPathIndex++;
                pathStarted = false;
            }
        }

        telemetry.addData("Current Path", currentPathIndex + "/" + paths.length);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
    }
}
package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Exact Pedro Path Fixed V2", group = "Auto")
public class ExactPedroPath extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Create the follower
        Follower follower = Constants.createFollower(hardwareMap,telemetry);

        // Set starting pose
        Pose startPose = new Pose(22.704103671706264, 121.29589632829374, Math.toRadians(180));
        follower.setStartingPose(startPose);

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        // Track the current pose manually
        Pose currentPose = startPose;

        // ---------------- PATH 1 (Linear) ----------------
        Path path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        currentPose,
                        new Pose(71.22246220302377, 72.15550755939526, Math.toRadians(180)),
                        false
                ))
                .build()
                .getPath(0);
        runPath(follower, path1);
        currentPose = new Pose(71.22246220302377, 72.15550755939526, Math.toRadians(134));

        // ---------------- PATH 2 (Reverse / Tangential) ----------------
        Path path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        currentPose,
                        new Pose(19.282937365010802, 84.28509719222463, Math.toRadians(132)),
                        true
                ))
                .build()
                .getPath(0);
        runPath(follower, path2);
        currentPose = new Pose(19.282937365010802, 84.28509719222463, Math.toRadians(132));

        // ---------------- PATH 3 (Reverse / Tangential) ----------------
        Path path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        currentPose,
                        new Pose(70.91144708423326, 72.15550755939526, Math.toRadians(180)),
                        true
                ))
                .build()
                .getPath(0);
        runPath(follower, path3);
        currentPose = new Pose(70.91144708423326, 72.15550755939526, Math.toRadians(180));

        // ---------------- PATH 4 (Reverse / Tangential) ----------------
        Path path4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        currentPose,
                        new Pose(18.038876889848815, 59.714902807775374, Math.toRadians(0)),
                        true
                ))
                .build()
                .getPath(0);
        runPath(follower, path4);
        currentPose = new Pose(18.038876889848815, 59.714902807775374, Math.toRadians(0));

        // ---------------- PATH 5 (Reverse / Tangential) ----------------
        Path path5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        currentPose,
                        new Pose(70.91144708423326, 72.46652267818574, Math.toRadians(180)),
                        true
                ))
                .build()
                .getPath(0);
        runPath(follower, path5);
        currentPose = new Pose(70.91144708423326, 72.46652267818574, Math.toRadians(180));

        // ---------------- PATH 6 (Reverse / Tangential) ----------------
        Path path6 = follower.pathBuilder()
                .addPath(new BezierLine(
                        currentPose,
                        new Pose(18.038876889848815, 35.76673866090713, Math.toRadians(0)),
                        true
                ))
                .build()
                .getPath(0);
        runPath(follower, path6);
        currentPose = new Pose(18.038876889848815, 35.76673866090713, Math.toRadians(0));

        // ---------------- PATH 7 (Linear) ----------------
        Path path7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        currentPose,
                        new Pose(70.91144708423326, 71.84449244060474, Math.toRadians(180)),
                        false
                ))
                .build()
                .getPath(0);
        runPath(follower, path7);

        telemetry.addLine("All paths complete");
        telemetry.update();
    }

    private void runPath(Follower follower, Path path) {
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
    }
}
package org.firstinspires.ftc.teamcode.Auto.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "9 Ball Auto - PedroPathing (Blue - 1)", group = "Autonomous")
@Configurable
public class PedroPathingStates extends OpMode {

    private Follower follower;
    private Paths paths;
    private int pathState = 0;
    private TelemetryManager panelsTelemetry;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap,telemetry);
        follower.setStartingPose(new Pose(33, 135, Math.toRadians(270)));
        paths = new Paths(follower);
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        follower.setPose(new Pose(33, 135, Math.toRadians(270)));
        follower.followPath(paths.preLoadShot, true);
        pathState = 0;
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        panelsTelemetry.debug("State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(paths.pickup1, true);
                    pathState = 1;
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot1, true);
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.pickup2, true);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.grab1, true);
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot2, true);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.getOut1, true);
                    pathState = 6;
                }
                break;
        }
    }

    public static class Paths {
        public PathChain preLoadShot, pickup1, shoot1, pickup2, grab1, shoot2, getOut1;

        // Custom constraint levels

        public Paths(Follower follower) {
            // preLoadShot: Runs at full global speed
            preLoadShot = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(33.000, 135.000),
                            new Pose(48.000, 84.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                    .build();

            // pickup1: Runs at full global speed
            pickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(48.000, 84.000),
                            new Pose(20.000, 84.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // shoot1: SLOWED DOWN for precision and to prevent rough stopping
            shoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(20.000, 84.000),
                            new Pose(48.000, 84.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // pickup2: Back to fast movement
            pickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(48.000, 84.000),
                            new Pose(43.000, 60.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            grab1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(43.000, 60.000),
                            new Pose(15.000, 59.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(15.000, 59.000),
                            new Pose(48.000, 84.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            getOut1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(48.000, 84.000),
                            new Pose(30.000, 80.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }
}
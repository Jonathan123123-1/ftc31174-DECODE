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

@Autonomous(name = "12 Ball Auto - PedroPathing (Red - 2)", group = "Autonomous")
@Configurable
public class PedroPathingRed2 extends OpMode {

    private Follower follower;
    private Paths paths;
    private int pathState = 0;
    private TelemetryManager panelsTelemetry;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(87, 8, Math.toRadians(90)));
        paths = new Paths(follower);
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
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
                    follower.followPath(paths.openGate1, true);
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot1, true);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.pickup2, true);
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.grab1, true);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.grab2, true);
                    pathState = 6;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot3, true);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.getOut1, true);
                    pathState = 8;
                }
                break;
        }
    }

    public static class Paths {
        public PathChain preLoadShot, pickup1, openGate1, shoot1, pickup2, grab1, shoot2, grab2, shoot3, getOut1;

        // Custom constraint levels
        public Paths(Follower follower) {
            // preLoadShot: Runs at full global speed
            preLoadShot = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(87.000, 8.000),
                            new Pose(96.000, 63.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            // pickup1: Runs at full global speed
            pickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(96.000, 63.000),
                            new Pose(128.000, 65.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // openGate1
            openGate1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(128.000, 65.000),
                            new Pose(130.000, 66.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                    .build();

            // shoot1: SLOWED DOWN for precision and to prevent rough stopping
            shoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(132.000, 66.000),
                            new Pose(89.000, 75.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))
                    .build();

            pickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(89.000, 75.000),
                            new Pose(96.000, 36.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            grab1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(96.000, 36.000),
                            new Pose(132.000, 36.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            shoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(132.000, 36.000),
                            new Pose(87.000, 10.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            grab2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(87.000, 10.000),
                            new Pose(133.000, 10.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            shoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(133.000, 10.000),
                            new Pose(86.000, 18.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            getOut1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(86.000, 18.000),
                            new Pose(120.000, 20.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }
    }
}
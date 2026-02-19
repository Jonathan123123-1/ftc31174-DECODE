package org.firstinspires.ftc.teamcode.Auto;

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

@Autonomous(name = "Auto Red Test - 2", group = "Autonomous")
@Configurable
public class AutoRedTest2 extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private int pathState;
    private Paths paths;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap,telemetry);

        // Robot starts here
        follower.setStartingPose(new Pose(120, 120, Math.toRadians(0)));

        paths = new Paths(follower);
        pathState = 0;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    /* ---------------- PATH DEFINITIONS ---------------- */

    public static class Paths {

        public PathChain preloadScore;
        public PathChain down1;
        public PathChain intakePickup1;
        public PathChain score1;
        public PathChain down2;
        public PathChain intakePickup2;
        public PathChain openGate;
        public PathChain score2;
        public PathChain getOut;

        public Paths(Follower follower) {

            preloadScore = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(120, 120),
                            new Pose(96, 96)
                    ))
                    .setLinearHeadingInterpolation(0, 0)
                    .build();

            down1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(96, 96),
                            new Pose(96, 84)
                    ))
                    .setLinearHeadingInterpolation(0, 0)
                    .build();

            intakePickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(96, 84),
                            new Pose(121, 84)
                    ))
                    .setLinearHeadingInterpolation(0, 0)
                    .build();

            score1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(121, 84),
                            new Pose(96, 96)
                    ))
                    .setLinearHeadingInterpolation(0, 0)
                    .build();

            down2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(96, 96),
                            new Pose(96, 60)
                    ))
                    .setLinearHeadingInterpolation(0, 0)
                    .build();

            intakePickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(96, 60),
                            new Pose(121, 60)
                    ))
                    .setLinearHeadingInterpolation(0, 0)
                    .build();

            openGate = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(121, 60),
                            new Pose(128, 70)
                    ))
                    .setLinearHeadingInterpolation(0, Math.toRadians(270))
                    .build();

            score2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(128, 70),
                            new Pose(96, 96)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                    .build();

            getOut = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(96, 96),
                            new Pose(115, 78)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(270), 0)
                    .build();
        }
    }

    /* ---------------- STATE MACHINE ---------------- */

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(paths.preloadScore);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.down1);
                    pathState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.intakePickup1);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.score1);
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.down2);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.intakePickup2);
                    pathState = 6;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.openGate);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.score2);
                    pathState = 8;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.getOut);
                    pathState = 9;
                }
                break;

            default:
                // Auto complete
                break;
        }
    }
}
package org.firstinspires.ftc.teamcode.Auto.Pathing;

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

@Autonomous(name = "Pedro Auto Testing", group = "Autonomous")
@Configurable
public class PedroAutonomous extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private int pathState;
    private Paths paths;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap,telemetry);

        // Robot is physically placed here
        follower.setStartingPose(new Pose(23, 120, Math.toRadians(180)));

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
        public PathChain score2;
        public PathChain down3;
        public PathChain intakePickup3;
        public PathChain score3;

        public Paths(Follower follower) {

            preloadScore = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(23, 120),
                            new Pose(48, 96)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(180)
                    )
                    .build();

            down1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(48, 96),
                            new Pose(48, 84)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            intakePickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(48, 84),
                            new Pose(18, 84)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            score1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(18, 84),
                            new Pose(48, 96)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            down2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(48, 96),
                            new Pose(48, 60)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            intakePickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(48, 60),
                            new Pose(18, 60)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            score2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(18, 60),
                            new Pose(48, 96)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            down3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(48, 96),
                            new Pose(48, 36)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            intakePickup3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(48, 36),
                            new Pose(18, 36)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            score3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(18, 36),
                            new Pose(48, 96)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
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
                    follower.followPath(paths.score2);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.down3);
                    pathState = 8;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.intakePickup3);
                    pathState = 9;
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.score3);
                    pathState = 10;
                }
                break;

            default:
                // Auto complete
                break;
        }
    }
}
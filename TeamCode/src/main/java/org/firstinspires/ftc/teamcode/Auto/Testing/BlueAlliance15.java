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

@Autonomous(name = "Blue Alliance 15 - Path Test", group = "Autonomous")
@Configurable // Panels
public class BlueAlliance15 extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap,telemetry);
        // Set starting pose based on the start of Path 1
        follower.setStartingPose(new Pose(33.000, 135.000, Math.toRadians(270)));

        paths = new Paths(follower); // Build paths

        pathState = 0;
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        // Start the first path
        follower.followPath(paths.preLoadShoot, true);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    // State machine for path execution
    private void autonomousPathUpdate() {
        if (follower.isBusy()) return; // Don't do anything while a path is running

        switch (pathState) {
            case 0: follower.followPath(paths.pickup1, true); pathState++; break;
            case 1: follower.followPath(paths.grab1, true); pathState++; break;
            case 2: follower.followPath(paths.openGate1, true); pathState++; break;
            case 3: follower.followPath(paths.shoot1, true); pathState++; break;
            case 4: follower.followPath(paths.openGate2, true); pathState++; break;
            case 5: follower.followPath(paths.shoot2, true); pathState++; break;
            case 6: follower.followPath(paths.openGate3, true); pathState++; break;
            case 7: follower.followPath(paths.shoot3, true); pathState++; break;
            case 8: follower.followPath(paths.pickup2, true); pathState++; break;
            case 9: follower.followPath(paths.grab2, true); pathState++; break;
            case 10: follower.followPath(paths.shootLast, true); pathState++; break;
            case 11: /* All paths complete */ pathState++; break;
        }
    }

    // Defines all the paths for the autonomous routine
    public static class Paths {
        public PathChain preLoadShoot, pickup1, grab1, openGate1, shoot1, openGate2, shoot2, openGate3, shoot3, pickup2, grab2, shootLast;

        public Paths(Follower follower) {
            preLoadShoot = follower.pathBuilder().addPath
                    (new BezierLine(new Pose(33.000, 135.000),
                            new Pose(48.000, 84.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                    .build();

            pickup1 = follower.pathBuilder().addPath
                    (new BezierLine(new Pose(48.000, 84.000),
                            new Pose(43.000, 60.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            grab1 = follower.pathBuilder().addPath
                    (new BezierLine(new Pose(43.000, 60.000),
                            new Pose(15.000, 60.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            openGate1 = follower.pathBuilder().addPath
                    (new BezierLine(new Pose(15.000, 60.000),
                            new Pose(18.000, 68.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                    .build();

            shoot1 = follower.pathBuilder().addPath
                    (new BezierLine(new Pose(18.000, 68.000),
                            new Pose(60.000, 68.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                    .build();

            openGate2 = follower.pathBuilder().addPath
                    (new BezierLine(new Pose(60.000, 68.000),
                            new Pose(13.000, 64.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(155))
                    .build();

            shoot2 = follower.pathBuilder().addPath
                    (new BezierLine(new Pose(13.000, 64.000),
                            new Pose(60.000, 69.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(155), Math.toRadians(155))
                    .build();

            openGate3 = follower.pathBuilder().addPath
                    (new BezierLine(new Pose(60.000, 69.000),
                            new Pose(13.000, 64.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(155), Math.toRadians(155))
                    .build();

            shoot3 = follower.pathBuilder().addPath
                    (new BezierLine(new Pose(13.000, 64.000),
                            new Pose(60.000, 69.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(155), Math.toRadians(180))
                    .build();

            pickup2 = follower.pathBuilder().addPath
                    (new BezierLine(new Pose(60.000, 69.000),
                            new Pose(45.000, 84.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            grab2 = follower.pathBuilder().addPath
                    (new BezierLine(new Pose(45.000, 84.000),
                            new Pose(20.000, 84.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shootLast = follower.pathBuilder().addPath
                    (new BezierLine(new Pose(20.000, 84.000),
                            new Pose(60.000, 107.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }
}

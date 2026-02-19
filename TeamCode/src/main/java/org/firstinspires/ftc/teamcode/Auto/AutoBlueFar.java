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

@Autonomous(name = "Auto Blue Far", group = "Autonomous")
@Configurable
public class AutoBlueFar extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private int pathState;
    private Paths paths;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap,telemetry);

        // ✅ Robot physically starts here
        follower.setStartingPose(new Pose(56, 9, Math.toRadians(180)));

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
        public PathChain up1;
        public PathChain intakePickup1;
        public PathChain shoot1;
        public PathChain up2;
        public PathChain intakePickup2;
        public PathChain gateOpen;
        public PathChain shoot2;
        public PathChain intakePickup3;
        public PathChain shoot3;
        public PathChain getOut;

        public Paths(Follower follower) {

            preloadScore = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56, 9),
                            new Pose(56, 15)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            up1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56, 15),
                            new Pose(56, 36)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            intakePickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56, 36),
                            new Pose(20, 36)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(20, 36),
                            new Pose(56, 15)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            up2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56, 15),
                            new Pose(56, 60)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            intakePickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56, 60),
                            new Pose(20, 60)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            gateOpen = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(20, 60),
                            new Pose(16, 70)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                    .build();

            shoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(16, 70),
                            new Pose(56, 15)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                    .build();

            intakePickup3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56, 15),
                            new Pose(10, 15)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(10, 15),
                            new Pose(56, 15)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            getOut = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56, 15),
                            new Pose(40, 20)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
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
                    follower.followPath(paths.up1);
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
                    follower.followPath(paths.shoot1);
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.up2);
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
                    follower.followPath(paths.gateOpen);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot2);
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
                    follower.followPath(paths.shoot3);
                    pathState = 10;
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.getOut);
                    pathState = 11;
                }
                break;

            default:
                // Auto complete
                break;
        }
    }
}
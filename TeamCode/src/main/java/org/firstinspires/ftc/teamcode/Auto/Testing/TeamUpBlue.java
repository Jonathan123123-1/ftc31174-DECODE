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

@Autonomous(name = "Team Up Blue - PedroPathing", group = "Autonomous")
@Configurable
public class TeamUpBlue extends OpMode {

    private Follower follower;
    private Paths paths;
    private int pathState = 0;
    private TelemetryManager panelsTelemetry;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap,telemetry);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));
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
                    follower.followPath(paths.backUp, true);
                    pathState = 1;
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.pickup1, true);
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
                    follower.followPath(paths.grab1, true);
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.pickup2, true);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot2, true);
                    pathState = 6;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.pickupGate, true);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot3, true);
                    pathState = 8;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.pickup4, true);
                    pathState = 9;
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot4, true);
                    pathState = 10;
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.getOut, true);
                    pathState = 11;
                }
                break;
        }
    }

    public static class Paths {

        public PathChain preLoadShot, backUp, pickup1, shoot1, grab1,
                pickup2, shoot2, pickupGate, shoot3, pickup4, shoot4, getOut;

        public Paths(Follower follower) {

            preLoadShot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56.00, 8.00),
                            new Pose(56.00, 15.00)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            backUp = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56.00, 15.00),
                            new Pose(56.00, 10.00)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            pickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56.00, 10.00),
                            new Pose(10.00, 10.00)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(10.00, 10.00),
                            new Pose(56.00, 15.00)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            grab1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56.00, 15.00),
                            new Pose(48.00, 36.00)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            pickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(48, 36),
                            new Pose(20, 36)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(20, 36),
                            new Pose(56, 15)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            pickupGate = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56, 15),
                            new Pose(10, 45)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();

            shoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(10, 45),
                            new Pose(56, 15)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            pickup4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56, 15),
                            new Pose(10, 25)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shoot4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(10, 25),
                            new Pose(56, 10)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            getOut = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56, 10),
                            new Pose(35, 45)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }
}

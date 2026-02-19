package org.firstinspires.ftc.teamcode.Auto.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;


@Autonomous(name = "PATH Testing rn", group = "Autonomous")
@Configurable // Panels
public class Blue12BallTeam extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private ElapsedTime StateTimer = new ElapsedTime();
    private ElapsedTime advanceTimer = new ElapsedTime();
    private boolean isWaiting = false;
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private int lastState = -1;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap,telemetry);
        follower.setStartingPose(new Pose(57, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }


    public static class Paths {
        public PathChain preLoadShot;
        public PathChain pickup1;
        public PathChain shoot1;
        public PathChain pickup2;
        public PathChain shoot2;
        public PathChain pickup3;
        public PathChain shoot3;

        public Paths(Follower follower) {
            preLoadShot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(57.000, 8.000),

                                    new Pose(67.000, 74.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                    .build();

            pickup1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(67.000, 74.000),
                                    new Pose(54.261, 55.717),
                                    new Pose(20.330, 62.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();


            shoot1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20.330, 62.000),

                                    new Pose(67.000, 74.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            pickup2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(67.000, 74.000),
                                    new Pose(58.188, 25.940),
                                    new Pose(20.000, 37.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20.000, 37.000),

                                    new Pose(64.000, 74.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            pickup3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(64.000, 74.000),
                                    new Pose(59.662, 1.338),
                                    new Pose(16.000, 15.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            shoot3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(16.000, 15.000),

                                    new Pose(60.000, 110.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();
        }
    }


    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follow(paths.preLoadShot);
                advanceAfterPath();
                break;

            case 1:
                follow(paths.pickup1);
                advanceAfterPath();
                break;

            case 2:
                follow(paths.shoot1);
                advanceAfterPath();
                break;

            case 3:
            //    follow(paths.pickup2);
            //    advanceAfterPath();
                break;

            case 4:
                follow(paths.shoot2);
                advanceAfterPath();
                break;

            case 5:
                follow(paths.pickup3);
                advanceAfterPath();
                break;

            case 6:
                follow(paths.shoot3);
                advanceAfterPath();
                break;

            default:
                break;
        }
        // Event markers will automatically trigger at their positions
        // Make sure to register NamedCommands in your RobotContainer
        return pathState;
    }

    private void follow (PathChain path) {
        follow(path, false);
    }

    public void follow(PathChain path, boolean holdPoint){
        if (pathState!= lastState) {
            follower.followPath(path, holdPoint);
            lastState = pathState;
        }
    }
    private void advance() {
        pathState++;
        StateTimer.reset();
        isWaiting = false;
    }

    private void advanceWhen(boolean cond) {
        if (cond) advance();
    }

    private void advanceAfterPath() {
        advanceWhen(!follower.isBusy());
    }

    private void advanceAfterPathWithMin(double minimumTime) {
        if (StateTimer.seconds() > minimumTime)
            advanceAfterPath();
    }

    private void advanceAfter(double seconds) {
        if (!isWaiting) advanceTimer.reset();
        isWaiting = true;
        advanceWhen(advanceTimer.seconds() > seconds);
        if (advanceTimer.seconds() > seconds)
            isWaiting = false;
    }
}


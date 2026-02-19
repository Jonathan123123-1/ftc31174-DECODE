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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable
public class AutoTesting extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Paths paths;

    private int pathState = 0;
    private final ElapsedTime waitTimer = new ElapsedTime();

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap,telemetry);

        // ✅ FIXED: starting pose now matches first path start
        follower.setStartingPose(new Pose(34, 135, Math.toRadians(270)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    private void autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                follower.followPath(paths.preLoadShot, true);
                pathState++;
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.pickup1, true);
                    pathState++;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.intakeIn1, true);
                    pathState++;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot1, true);
                    waitTimer.reset();
                    pathState++;
                }
                break;

            case 4:
                if (!follower.isBusy() && waitTimer.milliseconds() > paths.wait1) {
                    follower.followPath(paths.openGate1, true);
                    pathState++;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot2, true);
                    waitTimer.reset();
                    pathState++;
                }
                break;

            case 6:
                if (!follower.isBusy() && waitTimer.milliseconds() > paths.wait2) {
                    follower.followPath(paths.openGate2, true);
                    pathState++;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot3, true);
                    pathState++;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.intakeIn2, true);
                    pathState++;
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot4, true);
                    pathState++;
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.getOut, true);
                    pathState++;
                }
                break;

            default:
                // Auto complete
                break;
        }
    }

    // ------------------------------------------------------------------------

    public static class Paths {

        public final PathChain preLoadShot;
        public final PathChain pickup1;
        public final PathChain intakeIn1;
        public final PathChain shoot1;
        public final PathChain openGate1;
        public final PathChain shoot2;
        public final PathChain openGate2;
        public final PathChain shoot3;
        public final PathChain intakeIn2;
        public final PathChain shoot4;
        public final PathChain getOut;

        public final double wait1 = 600;
        public final double wait2 = 600;

        public Paths(Follower follower) {

            preLoadShot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(34, 135),
                            new Pose(48, 84)))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                    .build();

            pickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(48, 84),
                            new Pose(42, 60)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            intakeIn1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(42, 60),
                            new Pose(20, 60)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(20, 60),
                            new Pose(48, 84)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(136))
                    .build();

            openGate1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(48, 84),
                            new Pose(13, 61)))
                    .setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(136))
                    .build();

            shoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(13, 61),
                            new Pose(48, 84)))
                    .setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(136))
                    .build();

            openGate2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(48, 84),
                            new Pose(13, 61)))
                    .setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(136))
                    .build();

            shoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(13, 61),
                            new Pose(48, 84)))
                    .setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(180))
                    .build();

            intakeIn2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(48, 84),
                            new Pose(20, 84)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shoot4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(20, 84),
                            new Pose(48, 84)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            getOut = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(48, 84),
                            new Pose(48, 70)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }
}

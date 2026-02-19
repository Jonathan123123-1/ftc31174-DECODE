package org.firstinspires.ftc.teamcode.Auto.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "12 Ball Auto - PedroPathing (Blue - 2)", group = "Autonomous")
@Configurable
public class PedroPathingBlue2 extends OpMode {

    private Follower follower;
    private Paths paths;
    private int pathState = 0;
    private TelemetryManager panelsTelemetry;
    private DcMotorEx turretMotor;
    private DcMotorEx intakeMotor;


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(57, 8, Math.toRadians(90)));
        paths = new Paths(follower);

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(1.0);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
                    turretMotor.setTargetPosition(63);
                    turretMotor.setPower(0.4);
                    pathState = 1;
                }
                break;
            case 1:
                if (!turretMotor.isBusy()) {
                    turretMotor.setTargetPosition(63);
                    turretMotor.setPower(0.4);
                    follower.followPath(paths.pickup1, true);
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.openGate1, true);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot1, true);
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
                    follower.followPath(paths.grab1, true);
                    pathState = 6;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot2, true);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.grab2, true);
                    pathState = 8;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot3, true);
                    pathState = 9;
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.getOut1, true);
                    pathState = 10;
                }
        }
    }

    public static class Paths {
        public PathChain preLoadShot, pickup1, openGate1, shoot1, pickup2, grab1, shoot2, grab2, shoot3, getOut1;

        // Custom constraint levels
        private final PathConstraints slowConstraints =
                new PathConstraints(10, 2, Math.toRadians(180), Math.toRadians(180));

        public Paths(Follower follower) {
            // preLoadShot
            preLoadShot = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(57.000, 8.000),
                            new Pose(56.000, 36.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            // pickup1 first 3
            pickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(56.000, 36.000),
                            new Pose(23.000, 36.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // shoot1
            openGate1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(23.000, 36.000),
                            new Pose(60.000, 10.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // pickup2 for the next 3
            shoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(60.000, 10.000),
                            new Pose(56.000, 56.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // grab1 for the balls
            pickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(56.000, 56.000),
                            new Pose(25.000, 56.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // opening the gate
            grab1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(25.000, 56.000),
                            new Pose(30.000, 60.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                    .build();

            // shoot2
            shoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(30.000, 60.000),
                            new Pose(60.000, 10.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                    .build();

            // pickup2
            grab2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(60.000, 10.000),
                            new Pose(23.000, 10.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // shoot again
            shoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(20.000, 10.000),
                            new Pose(60.000, 10.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // get out
            getOut1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(60.000, 10.000),
                            new Pose(24.000, 15.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }
}

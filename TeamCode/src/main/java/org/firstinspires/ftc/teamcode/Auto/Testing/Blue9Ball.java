package org.firstinspires.ftc.teamcode.Auto.Testing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooterConstants.ShooterConstants;
import org.firstinspires.ftc.teamcode.shooterConstants.distanceLocalization;

import java.util.List;

@Autonomous(name = "Blue 9 Ball - FULL AUTO", group = "Autonomous")
public class Blue9Ball extends OpMode {

    // --- Core Systems ---
    private Follower follower;
    private Paths paths;
    private TelemetryManager panelsTelemetry;
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    // --- Mechanisms ---
    private DcMotorEx intakeMotor, shooterMotor, turretMotor;
    private Servo turretStopper, hoodServo;

    // --- Master State Machine ---
    private enum AutoState { 
        DRIVING_TO_FIRST_SHOT, SHOOTING_FIRST_BATCH,
        DRIVING_TO_SECOND_PICKUP, DRIVING_TO_THIRD_PICKUP, SHOOTING_SECOND_BATCH, 
        PARKING, DONE 
    }
    private AutoState currentState = AutoState.DRIVING_TO_FIRST_SHOT;

    // --- Sub-State Machines ---
    private enum StopperState { IDLE, PUSH, RETRACT }
    private StopperState stopperState = StopperState.IDLE;
    private int shotCount = 0;
    private ElapsedTime stopperTimer = new ElapsedTime();

    // --- Constants ---
    private final double TURRET_POWER = 0.8;
    private final double TURRET_TICKS_PER_DEGREE = 1.4936;
    private final int MAX_TURRET_TICKS = 360;
    private final int MIN_TURRET_TICKS = -360;
    private final int BLUE_GOAL_TAG_ID = 20;
    private final double INTAKE_VELOCITY = 1800; // Using consistent velocity from TeleOp

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        
        // THE FIX: Use the global follower from Constants.java for autonomous consistency
        follower = Constants.createFollower(hardwareMap);
        
        // Use the specific starting pose for this Autonomous routine
        follower.setStartingPose(new Pose(33, 135, Math.toRadians(270)));
        paths = new Paths(follower);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        turretStopper = hardwareMap.get(Servo.class, "turretStopper");
        hoodServo = hardwareMap.get(Servo.class, "turret_servo");

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        turretMotor.setDirection(DcMotorEx.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretStopper.setPosition(0);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Vision Initialization
        tagProcessor = new AprilTagProcessor.Builder().setLensIntrinsics(822.317, 822.317, 319.495, 242.502).build();
        visionPortal = new VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).addProcessor(tagProcessor).build();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        follower.followPath(paths.preLoadShot, true);
        currentState = AutoState.DRIVING_TO_FIRST_SHOT;
    }

    @Override
    public void loop() {
        follower.update();
        handleHybridAimingAndShooting(); // Always aim with the best available data
        runStateMachine();
        updateTelemetry();
    }

    private void handleHybridAimingAndShooting() {
        Pose robotPose = follower.getPose();
        double distanceToGoal; double turretAngleDeg;
        AprilTagDetection tag = getBlueTag();

        if (tag != null) {
            distanceToGoal = tag.ftcPose.range;
            double odometryAngleRad = distanceLocalization.getTargetTurretAngle(robotPose, false);
            turretAngleDeg = Math.toDegrees(odometryAngleRad) - tag.ftcPose.yaw;
        } else {
            distanceToGoal = distanceLocalization.getDistanceToGoal(robotPose, false);
            double odometryAngleRad = distanceLocalization.getTargetTurretAngle(robotPose, false);
            turretAngleDeg = Math.toDegrees(odometryAngleRad);
        }

        double targetRPM = ShooterConstants.targetRPM(distanceToGoal);
        double hoodPos = ShooterConstants.hoodPosition(distanceToGoal);
        shooterMotor.setVelocity(targetRPM);
        hoodServo.setPosition(Math.max(0.0, Math.min(1.0, hoodPos)));

        int targetPosition = (int) (turretAngleDeg * TURRET_TICKS_PER_DEGREE);
        int clampedPosition = Math.max(MIN_TURRET_TICKS, Math.min(targetPosition, MAX_TURRET_TICKS));
        turretMotor.setTargetPosition(clampedPosition);
        turretMotor.setPower(TURRET_POWER);
    }

    private void runStateMachine() {
        switch (currentState) {
            case DRIVING_TO_FIRST_SHOT:
                if (!follower.isBusy()) { currentState = AutoState.SHOOTING_FIRST_BATCH; }
                break;
            case SHOOTING_FIRST_BATCH:
                intakeMotor.setVelocity(INTAKE_VELOCITY);
                runShooterSequence(3);
                if (shotCount >= 3) {
                    shotCount = 0;
                    currentState = AutoState.DRIVING_TO_SECOND_PICKUP;
                    follower.followPath(paths.pickup1, true);
                }
                break;
            case DRIVING_TO_SECOND_PICKUP:
                intakeMotor.setVelocity(INTAKE_VELOCITY);
                if (!follower.isBusy()) { currentState = AutoState.DRIVING_TO_THIRD_PICKUP; follower.followPath(paths.grab1, true); }
                break;
            case DRIVING_TO_THIRD_PICKUP:
                intakeMotor.setVelocity(INTAKE_VELOCITY);
                if (!follower.isBusy()) { currentState = AutoState.SHOOTING_SECOND_BATCH; follower.followPath(paths.shoot2, true); }
                break;
            case SHOOTING_SECOND_BATCH:
                intakeMotor.setVelocity(INTAKE_VELOCITY);
                runShooterSequence(3);
                if (shotCount >= 3) {
                    shotCount = 0;
                    currentState = AutoState.PARKING;
                    follower.followPath(paths.getOut1, true);
                }
                break;
            case PARKING:
                if (!follower.isBusy()) { currentState = AutoState.DONE; }
                break;
            case DONE:
                intakeMotor.setPower(0); shooterMotor.setPower(0); turretMotor.setPower(0);
                break;
        }
    }

    private void runShooterSequence(int totalShots) {
        if (shotCount >= totalShots) { stopperState = StopperState.IDLE; turretStopper.setPosition(0); return; }
        switch (stopperState) {
            case IDLE: stopperState = StopperState.PUSH; stopperTimer.reset(); break;
            case PUSH: turretStopper.setPosition(0.5); if (stopperTimer.milliseconds() > 250) { stopperState = StopperState.RETRACT; stopperTimer.reset(); } break;
            case RETRACT: turretStopper.setPosition(0); if (stopperTimer.milliseconds() > 250) { shotCount++; stopperState = StopperState.IDLE; } break;
        }
    }

    private AprilTagDetection getBlueTag() {
        List<AprilTagDetection> detections = tagProcessor.getDetections();
        if(detections == null) return null;
        for (AprilTagDetection detection : detections) {
            if (detection.id == BLUE_GOAL_TAG_ID) return detection;
        }
        return null;
    }

    private void updateTelemetry(){ panelsTelemetry.debug("State", currentState); panelsTelemetry.debug("Shot Count", shotCount); panelsTelemetry.update(telemetry); }

    public static class Paths { 
        public PathChain preLoadShot, pickup1, shoot1, pickup2, grab1, shoot2, getOut1;
        public Paths(Follower f) {
            preLoadShot = f.pathBuilder()
                    .addPath(new BezierLine(new Pose(33,135),
                            new Pose(48,80)))
                    .setLinearHeadingInterpolation(Math.toRadians(270),
                            Math.toRadians(183)).build();

            pickup1 = f.pathBuilder()
                    .addPath(new BezierLine(new Pose(48,84),
                            new Pose(20,84)))
                    .setLinearHeadingInterpolation(Math.toRadians(180),
                            Math.toRadians(180)).build();

            shoot1 = f.pathBuilder()
                    .addPath(new BezierLine(new Pose(20,84),
                            new Pose(48,84)))
                    .setLinearHeadingInterpolation(Math.toRadians(180),
                            Math.toRadians(180)).build();

            pickup2 = f.pathBuilder()
                    .addPath(new BezierLine(new Pose(48,84),
                            new Pose(43,60)))
                    .setLinearHeadingInterpolation(Math.toRadians(180),
                            Math.toRadians(180)).build();

            grab1 = f.pathBuilder()
                    .addPath(new BezierLine(new Pose(43,60),
                            new Pose(15,59)))
                    .setLinearHeadingInterpolation(Math.toRadians(180),
                            Math.toRadians(180)).build();

            shoot2 = f.pathBuilder()
                    .addPath(new BezierLine(new Pose(15,59),
                            new Pose(48,84)))
                    .setLinearHeadingInterpolation(Math.toRadians(180),
                            Math.toRadians(180)).build();

            getOut1 = f.pathBuilder()
                    .addPath(new BezierLine(new Pose(48,84),
                            new Pose(30,80)))
                    .setLinearHeadingInterpolation(Math.toRadians(180),
                            Math.toRadians(180)).build();
        }
    }
}

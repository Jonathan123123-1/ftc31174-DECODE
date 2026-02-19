package org.firstinspires.ftc.teamcode.TeleOp.CompReady;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooterConstants.ShooterConstants;
import org.firstinspires.ftc.teamcode.shooterConstants.distanceLocalization;

import java.util.List;

@TeleOp(name = "NO TeleOp - BLUE", group = "COMPETITION")
public class CompetitionTeleOp_BLUE extends OpMode {

    // --- Hardware ---
    private DcMotorEx turretMotor, shooterMotor, intakeMotor;
    private Servo hoodServo, turretStopper;
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private IMU imu;
    private Follower follower;
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    // --- System State Machines ---
    private enum AimingState { IDLE, ACTIVE, HOMING }
    private AimingState aimingState = AimingState.IDLE;
    private enum DriveMode { ROBOT_CENTRIC, FIELD_CENTRIC }
    private DriveMode driveMode = DriveMode.ROBOT_CENTRIC;
    private enum SpeedMode { SLOW, FAST }
    private SpeedMode speedMode = SpeedMode.FAST;
    private enum StopperState { IDLE, PUSHING }
    private StopperState stopperState = StopperState.IDLE;
    private ElapsedTime stopperTimer = new ElapsedTime();

    // --- Control & Logic Variables ---
    private boolean lastToggleA1 = false, lastToggleX1 = false, lastToggleLB1 = false;
    private boolean lastBumperL2 = false, lastBumperR2 = false, lastBtnA2 = false;
    private boolean lastJoyUp2 = false, lastJoyDown2 = false;
    private boolean lastTriggerL2 = false, lastTriggerR2 = false;
    private boolean lastBtnX2 = false, lastBtnB2 = false;

    private double manualHoodOffset = 0.0;
    private double manualRpmOffset = 0.0;
    private int manualTurretOffset = 0;
    private String aimingMode = "OFF";

    // --- Constants ---
    private final double FAST_MODE_SPEED = 0.86;
    private final double SLOW_MODE_SPEED = 0.6;
    private final double TURRET_POWER = 0.8;
    private final int TURRET_HOME_POSITION = 0;
    private final double TURRET_TICKS_PER_DEGREE = 1.4936;
    private final int MAX_TURRET_TICKS = 180;
    private final int MIN_TURRET_TICKS = -270;
    private final double INTAKE_VELOCITY = 1800;
    private final int BLUE_GOAL_TAG_ID = 20;

    @Override
    public void init() {
        // Drivetrain & IMU
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        imu = hardwareMap.get(IMU.class, "imu");

        PinpointConstants correctedLocalizer = new PinpointConstants()
                .forwardPodY(183.16/2.54).strafePodX(-63.5/2.54)
                .distanceUnit(DistanceUnit.MM).hardwareMapName("pinpoint")
                .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
                .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
                .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

        follower = new FollowerBuilder(Constants.followerConstants, hardwareMap)
                .pinpointLocalizer(correctedLocalizer).pathConstraints(Constants.pathConstraints).mecanumDrivetrain(Constants.driveConstants).build();
        follower.setStartingPose(new Pose(139, 5, 0)); // Set Blue Alliance starting pose

        // Scoring Mechanisms
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        hoodServo = hardwareMap.get(Servo.class, "turret_servo");
        turretStopper = hardwareMap.get(Servo.class, "turretStopper");

        turretMotor.setDirection(DcMotorEx.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(TURRET_HOME_POSITION);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretStopper.setPosition(0); // Home position

        // Vision
        tagProcessor = new AprilTagProcessor.Builder().setLensIntrinsics(822.317, 822.317, 319.495, 242.502).build();
        visionPortal = new VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).addProcessor(tagProcessor).build();

        telemetry.addLine("Competition TeleOp Initialized. Ready for Battle.");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        handleDriving();
        handleOperatorControls();
        updateTelemetry();
    }

    private void handleDriving() {
        if (gamepad1.x && !lastToggleX1) driveMode = DriveMode.FIELD_CENTRIC;
        lastToggleX1 = gamepad1.x;
        if (gamepad1.b && !lastToggleA1) driveMode = DriveMode.ROBOT_CENTRIC;
        lastToggleA1 = gamepad1.b;
        if (gamepad1.a && !lastToggleLB1) speedMode = (speedMode == SpeedMode.FAST) ? SpeedMode.SLOW : SpeedMode.FAST;
        lastToggleLB1 = gamepad1.a;

        double maxSpeed = (speedMode == SpeedMode.FAST) ? FAST_MODE_SPEED : SLOW_MODE_SPEED;
        double y = -gamepad1.left_stick_y * maxSpeed, x = gamepad1.left_stick_x * maxSpeed, rx = gamepad1.right_stick_x * maxSpeed;

        if (driveMode == DriveMode.FIELD_CENTRIC) {
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw();
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            x = rotX; y = rotY;
        }

        double den = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftDrive.setPower((y + x + rx) / den); backLeftDrive.setPower((y - x + rx) / den);
        frontRightDrive.setPower((y - x - rx) / den); backRightDrive.setPower((y + x - rx) / den);
    }

    private void handleOperatorControls() {
        // Intake
        if (gamepad2.dpad_up) intakeMotor.setVelocity(INTAKE_VELOCITY);
        else if (gamepad2.dpad_down) intakeMotor.setVelocity(-INTAKE_VELOCITY);
        else if (gamepad2.dpad_left || gamepad2.dpad_right) intakeMotor.setPower(0);

        // Stopper
        if (gamepad2.a && !lastBtnA2) stopperState = StopperState.PUSHING;
        lastBtnA2 = gamepad2.a;
        handleStopper();

        // Aiming System Toggle
        if ((gamepad2.left_bumper && !lastBumperL2) || (gamepad2.right_bumper && !lastBumperR2)) {
            aimingState = (aimingState == AimingState.ACTIVE) ? AimingState.HOMING : AimingState.ACTIVE;
        }
        lastBumperL2 = gamepad2.left_bumper; lastBumperR2 = gamepad2.right_bumper;

        // Manual Overrides
        if (gamepad2.x && !lastBtnX2) manualTurretOffset -= 3;
        lastBtnX2 = gamepad2.x;
        if (gamepad2.b && !lastBtnB2) manualTurretOffset += 3;
        lastBtnB2 = gamepad2.b;
        if (gamepad2.right_stick_y < -0.5 && !lastJoyUp2) manualHoodOffset += 0.01;
        lastJoyUp2 = gamepad2.right_stick_y < -0.5;
        if (gamepad2.right_stick_y > 0.5 && !lastJoyDown2) manualHoodOffset -= 0.01;
        lastJoyDown2 = gamepad2.right_stick_y > 0.5;
        if (gamepad2.right_trigger > 0.5 && !lastTriggerR2) manualRpmOffset += 10;
        lastTriggerR2 = gamepad2.right_trigger > 0.5;
        if (gamepad2.left_trigger > 0.5 && !lastTriggerL2) manualRpmOffset -= 10;
        lastTriggerL2 = gamepad2.left_trigger > 0.5;

        // Aiming State Machine
        switch (aimingState) {
            case ACTIVE:
                handleHybridAimingAndShooting();
                break;
            case HOMING:
                shooterMotor.setPower(0);
                turretMotor.setTargetPosition(TURRET_HOME_POSITION);
                if (Math.abs(turretMotor.getCurrentPosition() - TURRET_HOME_POSITION) < 15) aimingState = AimingState.IDLE;
                break;
            case IDLE:
                turretMotor.setPower(0);
                shooterMotor.setPower(0);
                aimingMode = "OFF";
                break;
        }
    }

    private void handleStopper(){
        switch(stopperState){
            case PUSHING:
                turretStopper.setPosition(0.5);
                stopperTimer.reset();
                stopperState = StopperState.IDLE; // Non-blocking, will retract on next logic pass
                break;
            case IDLE:
                if(stopperTimer.milliseconds() > 250) {
                    turretStopper.setPosition(0);
                }
                break;
        }
    }

    private void handleHybridAimingAndShooting() {
        Pose robotPose = follower.getPose();
        double distanceToGoal; double turretAngleDeg;
        AprilTagDetection tag = getBlueTag();

        if (tag != null) {
            aimingMode = "VISION LOCK";
            distanceToGoal = tag.ftcPose.range;
            double odometryAngleRad = distanceLocalization.getTargetTurretAngle(robotPose, false);
            turretAngleDeg = Math.toDegrees(odometryAngleRad) - tag.ftcPose.yaw;
        } else {
            aimingMode = "ODOMETRY FALLBACK";
            distanceToGoal = distanceLocalization.getDistanceToGoal(robotPose, false);
            double odometryAngleRad = distanceLocalization.getTargetTurretAngle(robotPose, false);
            turretAngleDeg = Math.toDegrees(odometryAngleRad);
        }

        double targetRPM = ShooterConstants.targetRPM(distanceToGoal) + manualRpmOffset;
        double hoodPos = ShooterConstants.hoodPosition(distanceToGoal) + manualHoodOffset;
        shooterMotor.setVelocity(targetRPM);
        hoodServo.setPosition(clamp(hoodPos, 0.0, 1.0));

        int targetPosition = (int) (turretAngleDeg * TURRET_TICKS_PER_DEGREE) + manualTurretOffset;
        int clampedPosition = Math.max(MIN_TURRET_TICKS, Math.min(targetPosition, MAX_TURRET_TICKS));
        turretMotor.setTargetPosition(clampedPosition);
        turretMotor.setPower(TURRET_POWER);
    }

    private AprilTagDetection getBlueTag() {
        List<AprilTagDetection> detections = tagProcessor.getDetections();
        if (detections != null) {
            for (AprilTagDetection detection : detections) {
                if (detection.id == BLUE_GOAL_TAG_ID) return detection;
            }
        }
        return null;
    }

    private void updateTelemetry() {
        telemetry.addData("Drive Mode", driveMode + " | " + speedMode);
        telemetry.addData("Aiming State", aimingState + " | " + aimingMode);
        telemetry.addData("Hood Offset", manualHoodOffset);
        telemetry.addData("RPM Offset", manualRpmOffset);
        telemetry.addData("Turret Offset", manualTurretOffset);
        telemetry.update();
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}

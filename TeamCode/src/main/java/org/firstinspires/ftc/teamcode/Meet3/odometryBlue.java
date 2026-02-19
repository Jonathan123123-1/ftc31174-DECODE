package org.firstinspires.ftc.teamcode.Turret;

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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooterConstants.ShooterConstants;
import org.firstinspires.ftc.teamcode.shooterConstants.distanceLocalization;

import java.util.List;

@TeleOp(name = "100% Competition TeleOp - BLUE", group = "COMPETITION")
public class odometryBlue extends OpMode {

    // --- Hardware ---
    private DcMotorEx turretMotor, shooterMotor, intakeMotor;
    private Servo hoodServo, turretStopper;
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private IMU imu;
    private Follower follower;
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    // --- State Machine ---
    private enum SystemState { SYSTEM_OFF, SYSTEM_ACTIVE, HOMING }
    private SystemState systemState = SystemState.SYSTEM_OFF;

    // --- Control Constants ---
    private final double TURRET_TICKS_PER_DEGREE = 1.4936;
    private final double TURRET_POWER = 0.6;
    private final int TURRET_HOME_POSITION = 0;
    private final int MAX_TURRET_TICKS = 220;
    private final int MIN_TURRET_TICKS = -220;
    private final int BLUE_GOAL_TAG_ID = 20;
    private final double INTAKE_VELOCITY = 3500;
    private final double STOPPER_ENGAGED_POSITION = 0;
    private final double STOPPER_RELEASED_POSITION = 1.0;
    private final double RPM_ADJUSTMENT_STEP = 50;

    // --- Shooter PIDF ---
    // These need to be tuned for your specific shooter motor and setup.
    // P: Proportional - Increase for faster response, but too high can cause oscillation.
    // I: Integral - Helps eliminate steady-state error and maintain speed under load.
    // D: Derivative - Dampens response to prevent overshooting. Often 0 for velocity control.
    // F: Feedforward - Provides most of the power to get to the target velocity.
    private final double SHOOTER_P = 25.0;
    private final double SHOOTER_I = 5.0;
    private final double SHOOTER_D = 0.0;
    private final double SHOOTER_F = 15.0;

    // --- Logic Variables ---
    private boolean isSystemInitialized = false;
    private boolean lastRightBumper = false;
    private boolean lastGamepad2RightBumper = false;
    private boolean fieldOriented = false;
    private boolean lastToggleX = false;
    private boolean slowMode = false;
    private boolean lastGamepad1A = false;
    private String aimingMode = "OFF";
    private double targetRPM = 0;
    private double hoodPos = 0;
    private double rpmAdjustment = 0;
    private boolean lastGamepad2A = false;
    private boolean lastGamepad2B = false;
    private boolean lastGamepad2X = false;
    private boolean stopperEngaged = false;

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
                .forwardPodY(183.16/2.54)
                .strafePodX(-63.5/2.54)
                .distanceUnit(DistanceUnit.MM)
                .hardwareMapName("pinpoint")
                .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
                .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
                .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

        follower = new FollowerBuilder(Constants.followerConstants, hardwareMap)
                .pinpointLocalizer(correctedLocalizer)
                .pathConstraints(Constants.pathConstraints)
                .mecanumDrivetrain(Constants.driveConstants)
                .build();

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
        shooterMotor.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretStopper.setPosition(STOPPER_RELEASED_POSITION);

        // Vision
        tagProcessor = new AprilTagProcessor.Builder().setLensIntrinsics(822.317, 822.317, 319.495, 242.502).build();
        visionPortal = new VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).addProcessor(tagProcessor).build();

        telemetry.addLine("Competition TeleOp Initialized. Ready for battle.");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        handleDriving();
        handleSystemControls();
        handleGamepad2Controls();
        updateTelemetry();
    }

    private void handleGamepad2Controls() {
        handleIntake();
        handleTurretStopper();
        handleManualRpmAdjustment();
    }

    private void handleSystemControls() {
        boolean g1_rightBumper = gamepad1.right_bumper;
        boolean g2_rightBumper = gamepad2.right_bumper;

        if ((g1_rightBumper && !lastRightBumper) || (g2_rightBumper && !lastGamepad2RightBumper)) {
            if (!isSystemInitialized) {
                follower.setStartingPose(new Pose(144, 5, 0));
                isSystemInitialized = true;
                systemState = SystemState.SYSTEM_ACTIVE;
            } else {
                systemState = (systemState == SystemState.SYSTEM_ACTIVE) ? SystemState.HOMING : SystemState.SYSTEM_ACTIVE;
            }
        }
        lastRightBumper = g1_rightBumper;
        lastGamepad2RightBumper = g2_rightBumper;

        switch(systemState) {
            case SYSTEM_ACTIVE:
                handleHybridAimingAndShooting();
                break;
            case HOMING:
                turretMotor.setTargetPosition(TURRET_HOME_POSITION);
                shooterMotor.setPower(0);
                rpmAdjustment = 0;
                if (Math.abs(turretMotor.getCurrentPosition() - TURRET_HOME_POSITION) < 15) {
                    systemState = SystemState.SYSTEM_OFF;
                }
                break;
            case SYSTEM_OFF:
                turretMotor.setPower(0);
                shooterMotor.setPower(0);
                aimingMode = "OFF";
                rpmAdjustment = 0;
                break;
        }
    }

    private void handleHybridAimingAndShooting() {
        Pose robotPose = follower.getPose();
        double distanceToGoal;
        double turretAngleDeg;

        AprilTagDetection tag = getBlueTag();
        if (tag != null && tag.ftcPose != null) {
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

        double calculatedRPM = ShooterConstants.targetRPM(distanceToGoal);
        targetRPM = calculatedRPM + rpmAdjustment;
        hoodPos = ShooterConstants.hoodPosition(distanceToGoal);

        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        if (robotX >= 109 && robotX <= -56 && robotY >= -1 && robotY <= 22) {
            targetRPM = 1570;
            aimingMode = "ODOMETRY OVERRIDE - FAR";
        } else if (robotX >= 52 && robotX <= 105 && robotY >= 57 && robotY <= 95) {
            targetRPM = 1380;
            aimingMode = "ODOMETRY OVERRIDE - CLOSE";
        }

        shooterMotor.setVelocity(targetRPM);
        hoodServo.setPosition(hoodPos);

        int targetPosition = (int) (turretAngleDeg * TURRET_TICKS_PER_DEGREE);
        int clampedPosition = Math.max(MIN_TURRET_TICKS, Math.min(targetPosition, MAX_TURRET_TICKS));
        turretMotor.setTargetPosition(clampedPosition);
        turretMotor.setPower(TURRET_POWER);
    }

    private void handleIntake() {
        if (gamepad2.dpad_up) {
            intakeMotor.setVelocity(INTAKE_VELOCITY);
        } else if (gamepad2.dpad_down) {
            intakeMotor.setVelocity(-INTAKE_VELOCITY);
        } else if (gamepad2.dpad_left || gamepad2.dpad_right) {
            intakeMotor.setVelocity(0);
        }
    }

    private void handleTurretStopper() {
        if (gamepad2.x && !lastGamepad2X) {
            stopperEngaged = !stopperEngaged;
            turretStopper.setPosition(stopperEngaged ? STOPPER_ENGAGED_POSITION : STOPPER_RELEASED_POSITION);
        }
        lastGamepad2X = gamepad2.x;
    }

    private void handleManualRpmAdjustment() {
        if (gamepad2.a && !lastGamepad2A) {
            rpmAdjustment += RPM_ADJUSTMENT_STEP;
        }
        if (gamepad2.b && !lastGamepad2B) {
            rpmAdjustment -= RPM_ADJUSTMENT_STEP;
        }
        lastGamepad2A = gamepad2.a;
        lastGamepad2B = gamepad2.b;
    }

    private void handleDriving() {
        if (gamepad1.x && !lastToggleX) fieldOriented = !fieldOriented;
        lastToggleX = gamepad1.x;

        if (gamepad1.a && !lastGamepad1A) slowMode = !slowMode;
        lastGamepad1A = gamepad1.a;

        double maxSpeed = slowMode ? 0.4 : 1.0;
        double y = -gamepad1.left_stick_y * maxSpeed;
        double x = gamepad1.left_stick_x * maxSpeed;
        double rx = gamepad1.right_stick_x * maxSpeed;

        if (fieldOriented) {
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw();
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            x = rotX; y = rotY;
        }

        double den = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftDrive.setPower((y + x + rx) / den);
        backLeftDrive.setPower((y - x + rx) / den);
        frontRightDrive.setPower((y - x - rx) / den);
        backRightDrive.setPower((y + x - rx) / den);
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
        Pose robotPose = follower.getPose();
        telemetry.addLine("--- SYSTEM ---");
        telemetry.addData("System State", systemState);
        telemetry.addData("Drive Mode", (fieldOriented ? "FIELD" : "ROBOT") + (slowMode ? " | SLOW" : ""));
        telemetry.addLine("\n--- AIMING DATA ---");
        telemetry.addData("Aiming Mode", aimingMode);
        telemetry.addData("Target RPM", "%.1f", targetRPM);
        telemetry.addData("RPM Adjustment", "%.1f", rpmAdjustment);
        telemetry.addData("Hood Pos", "%.2f", hoodPos);
        telemetry.addLine("\n--- ROBOT POSE ---");
        telemetry.addData("Robot X", "%.2f", robotPose.getX());
        telemetry.addData("Robot Y", "%.2f", robotPose.getY());
        telemetry.addData("Robot Heading", "%.2f", Math.toDegrees(robotPose.getHeading()));
        telemetry.update();
    }
}

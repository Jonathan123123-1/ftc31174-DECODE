package org.firstinspires.ftc.teamcode.TeleOp.FullyDone;

import android.util.Size;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooterConstants.distanceLocalization;

import java.util.List;

@TeleOp(name = "Fully Turret Tracking - RED", group = "Robot")
public class FullyTurretTrackingRed extends LinearOpMode {

    // --- Core Systems ---
    private DcMotorEx turretMotor;
    private Follower follower;
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    private IMU imu; // For drive train
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    // --- Drive Control ---
    private double maxSpeed = 0.83;

    // --- Main State Machine ---
    private enum TrackingMode { DISABLED, CONSTANT_TRACK, HOMING }
    private TrackingMode trackingMode = TrackingMode.DISABLED;

    // --- Sub-States & Toggles ---
    private boolean useCameraAssist = false; // Toggled by left trigger in CONSTANT_TRACK mode
    private boolean manualTrackActive = false; // Toggled by bumpers in DISABLED mode
    private boolean lastRightTrigger = false, lastLeftTrigger = false, lastBumper = false;

    // --- Turret Constants ---
    private final int RED_GOAL_ID = 24;
    private final double TURRET_TICKS_PER_DEGREE = 113.78;
    private final double TURRET_POWER = 0.6;
    private final int TURRET_HOME_POSITION = 0;
    private final double MAX_TURRET_DEGREES = 90.0;
    private final double MIN_TURRET_DEGREES = -90.0;
    private double continuousTargetAngleDeg = 0.0;

    @Override
    public void runOpMode() {
        initializeHardware();
        waitForStart();

        while (opModeIsActive()) {
            // --- ALWAYS ON UPDATES ---
            follower.update();
            handleGamepadInput();

            // --- STATE-BASED LOGIC ---
            switch (trackingMode) {
                case CONSTANT_TRACK:
                    updateAiming(useCameraAssist);
                    break;
                case DISABLED:
                    if (manualTrackActive) {
                        updateAiming(true); // Manual mode always uses camera if available
                    } else {
                        turretMotor.setPower(0); // Relax the motor when not in use
                    }
                    break;
                case HOMING:
                    turretMotor.setTargetPosition(TURRET_HOME_POSITION);
                    turretMotor.setPower(TURRET_POWER);
                    if (Math.abs(turretMotor.getCurrentPosition() - TURRET_HOME_POSITION) < 20) {
                        trackingMode = TrackingMode.DISABLED;
                    }
                    break;
            }

            handleDriving();
            updateTelemetry();
        }
    }

    // ==================================================================================================
    //                                      CORE LOGIC
    // ==================================================================================================

    private void updateAiming(boolean useVision) {
        Pose robotPose = follower.getPose();
        double targetAngleRad = distanceLocalization.getTargetTurretAngle(robotPose, true); // True for Red Alliance

        if (useVision) {
            AprilTagDetection tag = getRedTag();
            if (tag != null) {
                double visionErrorRad = Math.toRadians(tag.ftcPose.yaw);
                targetAngleRad -= visionErrorRad; // Correct odometry with vision
            }
        }
        aimTurret(targetAngleRad);
    }

    // ==================================================================================================
    //                                      INITIALIZATION & HELPERS
    // ==================================================================================================

    private void initializeHardware() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        imu = hardwareMap.get(IMU.class, "imu");

        follower = Constants.createFollower(hardwareMap);

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        turretMotor.setDirection(DcMotorEx.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(TURRET_HOME_POSITION);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        tagProcessor = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).setCameraResolution(new Size(640, 480)).addProcessor(tagProcessor).build();

        telemetry.addLine("Initialization Complete.");
        telemetry.update();
    }

    private void handleGamepadInput() {
        boolean rightTrigger = gamepad2.right_trigger > 0.5;
        if (rightTrigger && !lastRightTrigger) {
            trackingMode = (trackingMode == TrackingMode.CONSTANT_TRACK) ? TrackingMode.HOMING : TrackingMode.CONSTANT_TRACK;
        }
        lastRightTrigger = rightTrigger;

        boolean leftTrigger = gamepad2.left_trigger > 0.5;
        if (leftTrigger && !lastLeftTrigger) {
            if (trackingMode == TrackingMode.CONSTANT_TRACK) {
                useCameraAssist = !useCameraAssist;
            }
        }
        lastLeftTrigger = leftTrigger;

        boolean bumper = gamepad2.left_bumper || gamepad2.right_bumper;
        if (bumper && !lastBumper) {
            if (trackingMode == TrackingMode.DISABLED) {
                manualTrackActive = !manualTrackActive;
                if (!manualTrackActive) {
                    trackingMode = TrackingMode.HOMING;
                }
            }
        }
        lastBumper = bumper;
    }

    private void handleDriving() {
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        drive(forward, right, rotate);
    }

    private void drive(double f, double r, double rot) {
        double fl = (f + r + rot) * maxSpeed;
        double fr = (f - r - rot) * maxSpeed;
        double bl = (f - r + rot) * maxSpeed;
        double br = (f + r - rot) * maxSpeed;
        frontLeftDrive.setPower(fl);
        frontRightDrive.setPower(fr);
        backLeftDrive.setPower(bl);
        backRightDrive.setPower(br);
    }

    private void aimTurret(double shortestAngleRad) {
        double shortestAngleDeg = Math.toDegrees(shortestAngleRad);
        while (Math.abs(shortestAngleDeg - continuousTargetAngleDeg) > 180) {
            if (shortestAngleDeg < continuousTargetAngleDeg) shortestAngleDeg += 360;
            else shortestAngleDeg -= 360;
        }
        continuousTargetAngleDeg = shortestAngleDeg;
        double clampedAngleDeg = Math.max(MIN_TURRET_DEGREES, Math.min(continuousTargetAngleDeg, MAX_TURRET_DEGREES));
        continuousTargetAngleDeg = clampedAngleDeg;
        int targetPosition = (int) (clampedAngleDeg * TURRET_TICKS_PER_DEGREE);
        turretMotor.setTargetPosition(targetPosition);
        turretMotor.setPower(TURRET_POWER);
    }

    private AprilTagDetection getRedTag() {
        List<AprilTagDetection> detections = tagProcessor.getDetections();
        if (detections != null) {
            for (AprilTagDetection tag : detections) {
                if (tag.id == RED_GOAL_ID) return tag;
            }
        }
        return null;
    }

    private void updateTelemetry() {
        telemetry.addLine("--- TURRET TRACKING TEST ---");
        telemetry.addData("Main Mode", trackingMode);
        if (trackingMode == TrackingMode.CONSTANT_TRACK) {
            telemetry.addData(" > Camera Assist", useCameraAssist ? "ON" : "OFF (Odometry Only)");
        } else if (trackingMode == TrackingMode.DISABLED) {
            telemetry.addData(" > Manual Mode", manualTrackActive ? "ACTIVE" : "INACTIVE");
        }
        telemetry.addData("Target Angle", "%.1f deg", continuousTargetAngleDeg);
        telemetry.update();
    }
}

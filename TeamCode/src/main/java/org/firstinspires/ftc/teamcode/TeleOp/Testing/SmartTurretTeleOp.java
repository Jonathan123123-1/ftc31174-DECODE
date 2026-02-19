package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Smart Turret TeleOp", group = "TEST")
public class SmartTurretTeleOp extends LinearOpMode {

    // --- Hardware ---
    private DcMotorEx turretMotor;
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private IMU imu;
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    // --- Turret State Machine ---
    private enum TurretState { IDLE, SCANNING_LEFT, SCANNING_RIGHT, LOCKED_ON_TARGET, HOMING }
    private TurretState turretState = TurretState.IDLE;

    // --- PID & Limits ---
    private double kP = 0.04;  // Aggressive gain for a strong lock
    private double kI = 0.015; // Integral gain to eliminate final offset
    private double kD = 0.002; // Derivative to dampen oscillation
    private double integralSum = 0;
    private double lastError = 0;

    private static final double DEADBAND_DEGREES = 0.5;
    private static final double MAX_POWER = 0.7;
    private static final int MAX_TICKS = 20;
    private static final int MIN_TICKS = -20;
    private final int BLUE_GOAL_TAG = 20;

    // --- Drive & Control Toggles ---
    private boolean fieldOriented = false;
    private boolean lastToggleX = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    @Override
    public void runOpMode() {
        initializeHardware();
        waitForStart();

        while (opModeIsActive()) {
            handleDriving();
            handleTurretLogic();
            updateTelemetry();
        }
    }

    private void handleTurretLogic() {
        // --- Bumper Input ---
        boolean leftBumper = gamepad2.left_bumper;
        if (leftBumper && !lastLeftBumper) {
            if (turretState == TurretState.IDLE) {
                turretState = TurretState.SCANNING_LEFT;
            } else { // If in any other state, go home
                turretState = TurretState.HOMING;
            }
        }
        lastLeftBumper = leftBumper;

        boolean rightBumper = gamepad2.right_bumper;
        if (rightBumper && !lastRightBumper) {
            if (turretState == TurretState.IDLE) {
                turretState = TurretState.SCANNING_RIGHT;
            } else { // If in any other state, go home
                turretState = TurretState.HOMING;
            }
        }
        lastRightBumper = rightBumper;

        // --- State Machine Execution ---
        AprilTagDetection tag = getBlueTag();

        // The core of Scan-and-Lock: If we see a tag, immediately switch to LOCKED_ON_TARGET
        if (tag != null && (turretState == TurretState.SCANNING_LEFT || turretState == TurretState.SCANNING_RIGHT)) {
            turretState = TurretState.LOCKED_ON_TARGET;
        }

        // If we were locked on but lose the tag, go home.
        if (tag == null && turretState == TurretState.LOCKED_ON_TARGET) {
            turretState = TurretState.HOMING;
        }

        switch (turretState) {
            case SCANNING_LEFT:
                runPIDToPosition(-20);
                break;
            case SCANNING_RIGHT:
                runPIDToPosition(20);
                break;
            case LOCKED_ON_TARGET:
                // This state uses the PID to drive yawError to zero
                runPIDToLock(tag.ftcPose.yaw);
                break;
            case HOMING:
                runPIDToPosition(0);
                // If we are close enough to home, go to idle
                if (Math.abs(turretMotor.getCurrentPosition()) < 3) {
                    turretState = TurretState.IDLE;
                }
                break;
            case IDLE:
                turretMotor.setPower(0);
                break;
        }
    }

    private void runPIDToPosition(int target) {
        int error = target - turretMotor.getCurrentPosition();
        runPID(error);
    }

    private void runPIDToLock(double yawError) {
        // In this case, the "error" is the yaw from the camera
        if (Math.abs(yawError) < DEADBAND_DEGREES) {
            turretMotor.setPower(0);
        } else {
            runPID(yawError);
        }
    }

    private void runPID(double error) {
        integralSum += error;
        double derivative = error - lastError;
        lastError = error;

        double power = (error * kP) + (integralSum * kI) + (derivative * kD);
        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

        int currentPosition = turretMotor.getCurrentPosition();
        if ((currentPosition >= MAX_TICKS && power > 0) || (currentPosition <= MIN_TICKS && power < 0)) {
            turretMotor.setPower(0);
        } else {
            turretMotor.setPower(power);
        }
    }

    // --- Standard Initialization and Helper methods ---
    private void initializeHardware() {
        // Drivetrain
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        imu = hardwareMap.get(IMU.class, "imu");

        // Turret Motor
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // Vision
        tagProcessor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(822.317, 822.317, 319.495, 242.502)
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(tagProcessor)
                .build();

        telemetry.addLine("Smart Turret Initialized");
        telemetry.update();
    }

    private void handleDriving() {
        // Field Oriented Toggle
        if (gamepad1.x && !lastToggleX) {
            fieldOriented = !fieldOriented;
        }
        lastToggleX = gamepad1.x;

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (fieldOriented) {
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw();
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            x = rotX;
            y = rotY;
        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftDrive.setPower((y + x + rx) / denominator);
        backLeftDrive.setPower((y - x + rx) / denominator);
        frontRightDrive.setPower((y - x - rx) / denominator);
        backRightDrive.setPower((y + x - rx) / denominator);
    }

    private AprilTagDetection getBlueTag() {
        List<AprilTagDetection> detections = tagProcessor.getDetections();
        if (detections != null) {
            for (AprilTagDetection detection : detections) {
                if (detection.id == BLUE_GOAL_TAG) {
                    return detection;
                }
            }
        }
        return null;
    }

    private void updateTelemetry() {
        telemetry.addData("Turret State", turretState);
        telemetry.addData("Turret Ticks", turretMotor.getCurrentPosition());
        telemetry.addData("Field Oriented", fieldOriented);
        telemetry.update();
    }
}

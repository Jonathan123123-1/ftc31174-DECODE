package org.firstinspires.ftc.teamcode.Auto;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "AutoRed1", group = "Autonomous")
public class AutoRed1 extends LinearOpMode {

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    // Motor constants
    private final double TICKS_PER_REV = 537.6; // REV HD 40:1
    private final double WHEEL_DIAMETER_INCH = 4.0;
    private final double COUNTS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCH);
    private final double DRIVE_POWER = 0.5;

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    private final int redGoalID = 24;

    @Override
    public void runOpMode() {

        // --- Hardware ---
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Reset encoders
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- Vision setup ---
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(tagProcessor)
                .build();

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        // --- Step 1: Move backward 48 inches with camera correction ---
        moveBackwardWithCamera(48);

        sleep(500); // small pause before next movement

        // --- Step 2: Strafe 24 inches right without camera ---
        strafeRight(24);

        telemetry.addLine("Autonomous routine complete");
        telemetry.update();
        sleep(1000);
    }

    private void moveBackwardWithCamera(double inches) {
        int moveCounts = (int)(inches * COUNTS_PER_INCH);

        frontLeftDrive.setTargetPosition(-moveCounts);
        frontRightDrive.setTargetPosition(-moveCounts);
        backLeftDrive.setTargetPosition(-moveCounts);
        backRightDrive.setTargetPosition(-moveCounts);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double Kp = 0.4; // Proportional gain for camera centering

        while (opModeIsActive() &&
                (frontLeftDrive.isBusy() || frontRightDrive.isBusy() ||
                        backLeftDrive.isBusy() || backRightDrive.isBusy())) {

            List<AprilTagDetection> detections = tagProcessor.getDetections();
            AprilTagDetection targetTag = null;

            for (AprilTagDetection tag : detections) {
                if (tag.id == redGoalID) {
                    targetTag = tag;
                    break;
                }
            }

            double strafe = 0;

            if (targetTag != null && targetTag.ftcPose != null) {
                double tagCenterX = targetTag.center.x;
                double camCenterX = 320;
                double xOffset = tagCenterX - camCenterX;
                double normalizedX = xOffset / camCenterX;
                strafe = Kp * normalizedX;
                strafe = Math.max(-0.3, Math.min(0.3, strafe));
            }

            // Apply strafe while moving backward
            frontLeftDrive.setPower(-DRIVE_POWER + strafe);
            frontRightDrive.setPower(-DRIVE_POWER - strafe);
            backLeftDrive.setPower(-DRIVE_POWER - strafe);
            backRightDrive.setPower(-DRIVE_POWER + strafe);

            telemetry.addData("FL", frontLeftDrive.getCurrentPosition());
            telemetry.addData("FR", frontRightDrive.getCurrentPosition());
            telemetry.addData("BL", backLeftDrive.getCurrentPosition());
            telemetry.addData("BR", backRightDrive.getCurrentPosition());
            telemetry.addData("Strafe Adjustment", strafe);
            telemetry.update();
        }

        stopMotors();
    }

    private void strafeRight(double inches) {
        // Reset encoders for pure strafe
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int moveCounts = (int)(inches * COUNTS_PER_INCH);

        // Encoder targets for pure right strafe
        frontLeftDrive.setTargetPosition(moveCounts);
        frontRightDrive.setTargetPosition(-moveCounts);
        backLeftDrive.setTargetPosition(-moveCounts);
        backRightDrive.setTargetPosition(moveCounts);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftDrive.setPower(0.5);
        frontRightDrive.setPower(0.5);
        backLeftDrive.setPower(0.5);
        backRightDrive.setPower(0.5);

        while (opModeIsActive() &&
                (frontLeftDrive.isBusy() || frontRightDrive.isBusy() ||
                        backLeftDrive.isBusy() || backRightDrive.isBusy())) {
            telemetry.addLine("Strafing right...");
            telemetry.update();
        }

        stopMotors();
    }

    private void stopMotors() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}
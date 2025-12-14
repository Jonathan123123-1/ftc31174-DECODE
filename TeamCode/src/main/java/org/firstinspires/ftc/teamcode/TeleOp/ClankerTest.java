package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "ClankerTest", group = "Robot")
public class ClankerTest extends LinearOpMode {

    // --- Drive Motors ---
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    // --- IMU ---
    private IMU imu;

    // --- Speed Control ---
    private double maxSpeed = 1.0;

    // --- Field-oriented toggle ---
    private boolean fieldOriented = false;
    private boolean lastTogglePressed = false;

    // --- Vision ---
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    private final int blueGoalID = 20; // Blue alliance tag
    private boolean cameraActive = false;
    private boolean lastLeftBumper = false;

    @Override
    public void runOpMode() {

        // --- Drive Motors ---
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- IMU ---
        imu = hardwareMap.get(IMU.class, "imu");

        // --- Vision ---
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

        telemetry.addLine("ClankerTest Initialized");
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // -------------------------
            // DRIVER 1 — TOGGLE FIELD ORIENTED
            // -------------------------
            boolean togglePressed = gamepad1.x;
            if (togglePressed && !lastTogglePressed) fieldOriented = !fieldOriented;
            lastTogglePressed = togglePressed;

            // -------------------------
            // DRIVER 1 — TOGGLE CAMERA
            // -------------------------
            if (gamepad1.left_bumper && !lastLeftBumper) {
                cameraActive = !cameraActive; // toggle camera on/off
            }
            lastLeftBumper = gamepad1.left_bumper;

            // -------------------------
            // DRIVER 1 — BASE MOVEMENT
            // -------------------------
            double forward = -gamepad1.left_stick_y;
            double right = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            if (gamepad1.a) maxSpeed = 0.5;
            if (gamepad1.b) maxSpeed = 1.0;

            // -------------------------
            // VISION PROCESSING
            // -------------------------
            List<AprilTagDetection> detections = null;
            AprilTagDetection targetTag = null;

            if (cameraActive) {
                detections = tagProcessor.getDetections();
                for (AprilTagDetection tag : detections) {
                    if (tag.id == blueGoalID) {
                        targetTag = tag;
                        break;
                    }
                }

                telemetry.addLine("Camera ACTIVE");
                telemetry.addData("Camera State", visionPortal.getCameraState());

                if (targetTag != null && targetTag.ftcPose != null) {
                    telemetry.addData("Tag Found", "YES");
                    telemetry.addData("Distance (in)", "%.1f", targetTag.ftcPose.range);
                    telemetry.addData("Angle (deg)", "%.1f", targetTag.ftcPose.bearing);

                    double tagCenterX = targetTag.center.x;
                    double tagCenterY = targetTag.center.y;
                    double camCenterX = 320;
                    double camCenterY = 240;

                    double xOffset = tagCenterX - camCenterX;
                    double yOffset = tagCenterY - camCenterY;

                    double normalizedX = xOffset / camCenterX;
                    double normalizedY = yOffset / camCenterY;

                    telemetry.addData("X Offset", "%.2f", normalizedX);
                    telemetry.addData("Y Offset", "%.2f", normalizedY);

                    double threshold = 0.5;
                    if (Math.abs(normalizedX) < threshold && Math.abs(normalizedY) < threshold) {
                        telemetry.addLine("READY TO SHOOT!");
                    } else {
                        telemetry.addLine("NOT READY");
                    }

                    // -------------------------
                    // RIGHT BUMPER → AUTO AIM
                    // -------------------------
                    if (gamepad1.right_bumper) {
                        telemetry.addLine("AUTO AIM ACTIVE");

                        double Kp = 0.4; // smooth proportional correction
                        right = normalizedX * Kp;
                        forward = normalizedY * Kp;

                        telemetry.addLine("AUTO MOVING TO CENTER");
                    }

                } else {
                    telemetry.addLine("Tag not detected");
                }

            } else {
                telemetry.addLine("Camera OFF");
            }

            // -------------------------
            // FIELD-ORIENTED ADJUSTMENT
            // -------------------------
            if (fieldOriented) {
                double heading = imu.getRobotYawPitchRollAngles().getYaw();
                double temp = forward * Math.cos(heading) + right * Math.sin(heading);
                right = -forward * Math.sin(heading) + right * Math.cos(heading);
                forward = temp;
            }

            // -------------------------
            // DRIVE
            // -------------------------
            drive(forward, right, rotate);

            // -------------------------
            // TELEMETRY
            // -------------------------
            telemetry.addData("Speed Mode", maxSpeed);
            telemetry.addData("Field Oriented", fieldOriented ? "ON" : "OFF");
            telemetry.addData("FL Power", frontLeftDrive.getPower());
            telemetry.addData("FR Power", frontRightDrive.getPower());
            telemetry.addData("BL Power", backLeftDrive.getPower());
            telemetry.addData("BR Power", backRightDrive.getPower());
            telemetry.update();
        }
    }

    private void drive(double forward, double right, double rotate) {
        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double bl = forward - right + rotate;
        double br = forward + right - rotate;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        frontLeftDrive.setPower(fl * maxSpeed);
        frontRightDrive.setPower(fr * maxSpeed);
        backLeftDrive.setPower(bl * maxSpeed);
        backRightDrive.setPower(br * maxSpeed);
    }
}
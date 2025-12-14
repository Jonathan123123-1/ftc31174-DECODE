package org.firstinspires.ftc.teamcode.Auto.AprilTags;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name="Red Alliance Autonomous Goal Tracking", group="Vision")
public class RedAllianceAutonomous extends LinearOpMode {

    // --- Drive Motors ---
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    // --- Vision ---
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    private final int redGoalID = 24;

    // --- Drive auto-align constants ---
    double KpStrafe = 0.4;
    double KpRotate = 0.2;
    double minPower = 0.15;

    @Override
    public void runOpMode() {

        // --- Motors ---
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // --- Vision ---
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tagProcessor)
                .build();

        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            List<AprilTagDetection> detections = tagProcessor.getDetections();
            AprilTagDetection targetTag = null;

            // Look for the red goal tag
            for (AprilTagDetection tag : detections) {
                if (tag.id == redGoalID) {
                    targetTag = tag;
                    break;
                }
            }

            double forward = 0; // stay in place
            double right = 0;
            double rotate = 0;

            if (targetTag != null && targetTag.ftcPose != null) {
                double tagCenterX = targetTag.center.x;
                double camCenterX = 320;

                // Compute normalized X offset for strafing and rotation
                double xOffset = tagCenterX - camCenterX;
                double normalizedX = xOffset / camCenterX;

                right = normalizedX * KpStrafe;
                rotate = normalizedX * KpRotate;

                if (Math.abs(right) < minPower && Math.abs(normalizedX) > 0.05)
                    right = Math.signum(right) * minPower;
                if (Math.abs(rotate) < minPower && Math.abs(normalizedX) > 0.05)
                    rotate = Math.signum(rotate) * minPower;

                telemetry.addData("Tag Found", targetTag.id);
                telemetry.addData("Strafe Power", right);
                telemetry.addData("Rotate Power", rotate);

            } else {
                telemetry.addLine("No red goal detected");
            }

            // --- Drive robot ---
            drive(forward, right, rotate);
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
            fl /= max; fr /= max; bl /= max; br /= max;
        }

        frontLeftDrive.setPower(fl);
        frontRightDrive.setPower(fr);
        backLeftDrive.setPower(bl);
        backRightDrive.setPower(br);
    }
}
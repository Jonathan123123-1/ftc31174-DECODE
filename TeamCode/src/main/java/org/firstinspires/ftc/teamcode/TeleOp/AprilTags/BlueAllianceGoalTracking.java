package org.firstinspires.ftc.teamcode.TeleOp.AprilTags;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "Blue Alliance Goal Tracking (Vision Only)", group = "Vision")
public class BlueAllianceGoalTracking extends LinearOpMode {

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    // Blue Alliance goal tag ID
    private final int blueGoalID = 20;

    @Override
    public void runOpMode() {

        // AprilTag processor
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        // VisionPortal (Logitech C270)
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480)) // Best for C270
                .addProcessor(tagProcessor)
                .build();

        telemetry.addLine("Vision initialized");
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addLine("=== C270 Blue Goal Tracking ===");
            telemetry.addData("Camera State", visionPortal.getCameraState());

            if (gamepad1.left_bumper) { // Only process camera when left bumper is pressed

                List<AprilTagDetection> detections = tagProcessor.getDetections();
                AprilTagDetection targetTag = null;

                // Find blue goal tag
                for (AprilTagDetection tag : detections) {
                    if (tag.id == blueGoalID) {
                        targetTag = tag;
                        break;
                    }
                }

                if (targetTag != null && targetTag.ftcPose != null) {

                    // Tag info
                    telemetry.addData("Tag Found", "YES");
                    telemetry.addData("Tag ID", targetTag.id);
                    telemetry.addData("Distance (in)", "%.1f", targetTag.ftcPose.range);
                    telemetry.addData("Angle (deg)", "%.1f", targetTag.ftcPose.bearing);

                    // X/Y offset calculations
                    double tagCenterX = targetTag.center.x;
                    double tagCenterY = targetTag.center.y;
                    double camCenterX = 320; // 640 / 2
                    double camCenterY = 240; // 480 / 2

                    double xOffset = tagCenterX - camCenterX;
                    double yOffset = tagCenterY - camCenterY;

                    double normalizedX = xOffset / camCenterX; // -1 to 1
                    double normalizedY = yOffset / camCenterY; // -1 to 1

                    telemetry.addData("Tag Center X", "%.1f", tagCenterX);
                    telemetry.addData("Tag Center Y", "%.1f", tagCenterY);
                    telemetry.addData("X Offset (px)", "%.1f", xOffset);
                    telemetry.addData("Y Offset (px)", "%.1f", yOffset);
                    telemetry.addData("Normalized X", "%.2f", normalizedX);
                    telemetry.addData("Normalized Y", "%.2f", normalizedY);

                    // READY TO SHOOT check
                    double threshold = 0.5; // adjust for tolerance
                    if (Math.abs(normalizedX) < threshold && Math.abs(normalizedY) < threshold) {
                        telemetry.addLine("READY TO SHOOT!");
                    } else {
                        telemetry.addLine("NOT READY");
                    }

                } else {
                    telemetry.addData("Tag Found", "NO");
                    telemetry.addLine("Searching for blue goal...");
                }

            } else {
                // Left bumper not pressed
                telemetry.addLine("Camera inactive. Hold LEFT BUMPER to track.");
            }

            telemetry.update();
        }
    }
}
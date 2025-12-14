package org.firstinspires.ftc.teamcode.TeleOp.AprilTags;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name="Red Alliance Goal Tracking", group="Vision")
public class RedAllianceGoalTracking extends LinearOpMode {

    DcMotor turretMotor;

    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    int redGoalID = 24;

    double Kp = 0.01, Ki = 0.0, Kd = 0.0;
    double integral = 0, lastError = 0;

    @Override
    public void runOpMode() {

        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");

        // New AprilTag Processor — Latest FTC SDK
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

            // Find tag ID 24 (red goal)
            for (AprilTagDetection tag : detections) {
                if (tag.id == redGoalID) {
                    targetTag = tag;
                    break;
                }
            }

            if (targetTag != null && targetTag.ftcPose != null) {

                double distanceInches = targetTag.ftcPose.range;
                double angleDegrees = targetTag.ftcPose.bearing;   // horizontal angle

                // PID
                double error = angleDegrees;
                integral += error;
                double derivative = error - lastError;
                lastError = error;

                double pidOutput = Kp * error + Ki * integral + Kd * derivative;
                pidOutput = Math.max(-1, Math.min(1, pidOutput));

                turretMotor.setPower(pidOutput);

                telemetry.addData("Detected Tag ID", targetTag.id);
                telemetry.addData("Distance (in)", distanceInches);
                telemetry.addData("Angle (deg)", angleDegrees);
                telemetry.addData("PID Output", pidOutput);

            } else {
                turretMotor.setPower(0);
                telemetry.addLine("No red goal detected");
            }

            telemetry.update();
        }
    }
}
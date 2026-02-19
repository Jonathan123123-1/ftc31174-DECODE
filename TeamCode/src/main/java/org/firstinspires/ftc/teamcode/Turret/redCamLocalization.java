package org.firstinspires.ftc.teamcode.Turret;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Camera Turret Lock(Red)", group = "Turret")
public class redCamLocalization extends LinearOpMode {

    // --- Hardware ---
    private DcMotorEx turretMotor;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // --- Constants ---
    private final int RED_GOAL_TAG_ID = 24;
    // From your odometry class: (8192 Ticks * 5) / 360 Degrees = 113.78
    private final double TURRET_TICKS_PER_DEGREE = 113.78;
    private final double TURRET_POWER = 0.7;

    @Override
    public void runOpMode() {
        // --- Initialization ---
        telemetry.addLine("Initializing...");

        // Turret Motor
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        turretMotor.setDirection(DcMotorEx.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(TURRET_POWER);

        // Vision
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Ready! Press Play to start camera locking.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            AprilTagDetection targetTag = null;

            // --- Find the Target Tag ---
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == RED_GOAL_TAG_ID) {
                    targetTag = detection;
                    break; // Found the tag, no need to keep searching
                }
            }

            // --- Control Logic ---
            if (targetTag != null) {
                // The tag is visible. Get the yaw error.
                double yawErrorDegrees = targetTag.ftcPose.yaw;

                // Calculate how many ticks to move to correct the error.
                double errorCorrectionTicks = yawErrorDegrees * TURRET_TICKS_PER_DEGREE;

                // Calculate the new target position by adjusting from the current position.
                int newTargetPosition = turretMotor.getCurrentPosition() - (int)errorCorrectionTicks;

                // Command the motor to the new position.
                turretMotor.setTargetPosition(newTargetPosition);
                turretMotor.setPower(TURRET_POWER);

                // --- Telemetry ---
                telemetry.addLine("TARGET ACQUIRED");
                telemetry.addData("Tag Yaw (Error)", "%.2f degrees", yawErrorDegrees);
                telemetry.addData("Current Ticks", turretMotor.getCurrentPosition());
                telemetry.addData("Target Ticks", newTargetPosition);

            } else {
                // The tag is not visible. Hold the last position.
                turretMotor.setTargetPosition(turretMotor.getTargetPosition());
                telemetry.addLine("SEARCHING for Tag ID: " + RED_GOAL_TAG_ID);
            }

            telemetry.update();
        }
    }
}

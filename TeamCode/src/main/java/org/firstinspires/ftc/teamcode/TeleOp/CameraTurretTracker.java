package org.firstinspires.ftc.teamcode.TeleOp;

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

@TeleOp(name = "Camera Turret Tracker (Blue)", group = "TEST")
public class CameraTurretTracker extends LinearOpMode {

    // --- Hardware ---
    private DcMotorEx turretMotor;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // --- Constants ---
    private final int BLUE_GOAL_TAG_ID = 20;
    // Corrected for GoBilda 312 RPM motor (537.7 ticks/rev) @ 1:1
    private final double TICKS_PER_DEGREE = 1.4936;
    private final double TURRET_POWER = 0.6;

    // --- Logic ---
    private double continuousTargetAngleDeg = 0.0;

    @Override
    public void runOpMode() {

        // --- Initialization ---
        try {
            turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
            turretMotor.setDirection(DcMotorEx.Direction.REVERSE);
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setTargetPosition(0);
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turretMotor.setPower(TURRET_POWER);
        } catch (Exception e) {
            telemetry.addLine("ERROR: Could not find or cast 'turret_motor' to DcMotorEx.");
            telemetry.update();
            sleep(10000);
            return;
        }

        // Vision Initialization
        aprilTag = new AprilTagProcessor.Builder().setDrawTagID(true).build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Camera Turret Tracker Ready.");
        telemetry.update();

        waitForStart();
        
        // Initialize the continuous angle to the turret's starting position
        continuousTargetAngleDeg = turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;

        while (opModeIsActive()) {
            AprilTagDetection targetTag = null;

            // --- Find the Target Tag ---
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == BLUE_GOAL_TAG_ID) {
                    targetTag = detection;
                    break;
                }
            }

            // --- Control Logic ---
            if (targetTag != null) {
                // 1. Get the camera's yaw error.
                double yawErrorDegrees = targetTag.ftcPose.yaw;

                // 2. Calculate the turret's new target angle based on the error.
                double shortestAngleDeg = (turretMotor.getCurrentPosition() / TICKS_PER_DEGREE) - yawErrorDegrees;

                // --- ANGLE UNWRAPPING: Prevents the 360-degree spin ---
                while (Math.abs(shortestAngleDeg - continuousTargetAngleDeg) > 180) {
                    if (shortestAngleDeg < continuousTargetAngleDeg) {
                        shortestAngleDeg += 360;
                    } else {
                        shortestAngleDeg -= 360;
                    }
                }
                continuousTargetAngleDeg = shortestAngleDeg;

                // 3. Command the motor to the new, unwrapped position.
                int newTargetPosition = (int) (continuousTargetAngleDeg * TICKS_PER_DEGREE);
                turretMotor.setTargetPosition(newTargetPosition);
                turretMotor.setPower(TURRET_POWER);

                telemetry.addLine("--- TARGET ACQUIRED ---");
                telemetry.addData("Yaw Error", "%.2f deg", yawErrorDegrees);

            } else {
                // If we can't see the tag, RUN_TO_POSITION will hold the last target automatically.
                telemetry.addLine("--- SEARCHING FOR TAG " + BLUE_GOAL_TAG_ID + " ---");
            }

            // --- Telemetry ---
            telemetry.addData("Target Angle", "%.2f", continuousTargetAngleDeg);
            telemetry.addData("Turret Target Ticks", turretMotor.getTargetPosition());
            telemetry.addData("Turret Current Ticks", turretMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}

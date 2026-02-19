package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "TurretFieldLocked_BLUE", group = "TEST")
public class TurretFieldLocked extends LinearOpMode {

    // ---------------- HARDWARE ----------------
    private DcMotorEx turretMotor;

    // ---------------- VISION ----------------
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    private final int BLUE_GOAL_TAG = 20;

    // ---------------- PID-CONTROLLER & LIMITS ----------------
    private static final double kP = 0.04;
    private static final double kI = 0.015;
    private static final double kD = 0.002;
    private double integralSum = 0;
    private double lastError = 0;

    private static final double DEADBAND_DEGREES = 0.5; 
    private static final double MAX_POWER = 0.7;
    private static final int MAX_TICKS = 20;
    private static final int MIN_TICKS = -20;

    @Override
    public void runOpMode() {

        // ---------------- INIT ----------------
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // Vision initialization
        double fx = 822.317; // Placeholder focal length x
        double fy = 822.317; // Placeholder focal length y
        double cx = 319.495; // Placeholder principal point x
        double cy = 242.502; // Placeholder principal point y

        tagProcessor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(fx, fy, cx, cy) // Corrects for camera orientation
                .setDrawTagID(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640,480))
                .addProcessor(tagProcessor)
                .build();

        telemetry.addLine("Camera Turret Lock READY (BLUE)");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {

            // ---------------- CAMERA-LOCK AIM ----------------
            AprilTagDetection tag = getBlueTag();

            if(tag != null && tag.ftcPose != null) {
                double yawError = tag.ftcPose.yaw;

                if (Math.abs(yawError) < DEADBAND_DEGREES) {
                    turretMotor.setPower(0);
                    integralSum = 0; 
                    lastError = yawError; // Keep lastError updated
                } else {
                    integralSum += yawError;
                    double derivative = yawError - lastError;
                    lastError = yawError;

                    // Calculate power using full PID terms
                    double power = (yawError * kP) + (integralSum * kI) + (derivative * kD);
                    power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

                    int currentPosition = turretMotor.getCurrentPosition();
                    if ((currentPosition >= MAX_TICKS && power > 0) || (currentPosition <= MIN_TICKS && power < 0)) {
                        turretMotor.setPower(0);
                    } else {
                        turretMotor.setPower(power);
                    }
                }
                telemetry.addLine("--- TARGET ACQUIRED ---");
                telemetry.addData("Yaw Error", "%.2f deg", yawError);

            } else {
                turretMotor.setPower(0);
                integralSum = 0;
                lastError = 0; // Reset error when tag is lost
                telemetry.addLine("--- SEARCHING FOR TAG " + BLUE_GOAL_TAG + " ---");
            }

            // ---------------- TELEMETRY ----------------
            telemetry.addData("Motor Power", turretMotor.getPower());
            telemetry.addData("Current Ticks", turretMotor.getCurrentPosition());
            telemetry.update();
        }
    } 

    private AprilTagDetection getBlueTag() {
        List<AprilTagDetection> detections = tagProcessor.getDetections();
        if (detections == null) return null;
        for(AprilTagDetection tag : detections) {
            if(tag.id == BLUE_GOAL_TAG) {
                return tag;
            }
        }
        return null;
    }
}

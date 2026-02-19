package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Test: Limelight Shooting", group = "TEST")
public class LimelightShootingTesting extends LinearOpMode {

    // --- Hardware ---
    private DcMotorEx shooterMotor;
    private DcMotorEx intakeMotor;
    private Limelight3A limelight;

    // --- Constants ---
    private static final double INTAKE_VELOCITY = 1800; // Velocity for the intake motor
    private static final int RPM_INCREMENT_MAJOR = 100;
    private static final int RPM_ADJUSTMENT_MINOR = 50;
    private static final int BLUE_PIPELINE = 8;

    // --- Shooter PIDF ---
    // TODO: Tune these values for your specific shooter motor and setup
    private static final double SHOOTER_P = 75.0;
    private static final double SHOOTER_I = 0.0;
    private static final double SHOOTER_D = 0.0;
    private static final double SHOOTER_F = 11.7; // kF = 32767 / max_rpm_velocity

    // TODO: Tune these values for your specific robot and target
    private static final double CAMERA_HEIGHT_INCHES = 12.0; // Height of the camera lens from the floor
    private static final double TARGET_HEIGHT_INCHES = 29.5; // Height of the AprilTag center from the floor
    private static final double CAMERA_MOUNT_ANGLE_DEGREES = 0.0; // Camera mounting angle

    // --- Control Logic ---
    private double targetRPM = 0; // Initial RPM
    private boolean lastGamepadX = false;
    private boolean lastGamepadA = false;
    private boolean lastRightTrigger = false;
    private boolean lastLeftTrigger = false;

    @Override
    public void runOpMode() {

        // --- Initialization ---
        try {
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
            limelight = hardwareMap.get(Limelight3A.class, "limelight");

            shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            shooterMotor.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);

            intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            limelight.pipelineSwitch(BLUE_PIPELINE);
            limelight.start();

        } catch (Exception e) {
            telemetry.addLine("ERROR: Could not initialize hardware.");
            telemetry.addLine(e.getMessage());
            telemetry.update();
            sleep(10000);
            return;
        }

        telemetry.addLine("Limelight Shooting Test Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- Intake Control ---
            if (gamepad1.dpad_up) {
                intakeMotor.setVelocity(-INTAKE_VELOCITY); // Run intake in reverse
            } else {
                intakeMotor.setVelocity(0);
            }

            // --- Shooter RPM Adjustment ---
            // Major adjustments with A and X buttons
            if (gamepad1.x && !lastGamepadX) {
                targetRPM += RPM_INCREMENT_MAJOR;
            }
            if (gamepad1.a && !lastGamepadA) {
                targetRPM -= RPM_INCREMENT_MAJOR;
            }

            // Minor adjustments with triggers
            boolean rightTriggerPressed = gamepad1.right_trigger > 0.5;
            boolean leftTriggerPressed = gamepad1.left_trigger > 0.5;

            if (rightTriggerPressed && !lastRightTrigger) {
                targetRPM += RPM_ADJUSTMENT_MINOR;
            }
            if (leftTriggerPressed && !lastLeftTrigger) {
                targetRPM -= RPM_ADJUSTMENT_MINOR;
            }

            // Update last button states
            lastGamepadX = gamepad1.x;
            lastGamepadA = gamepad1.a;
            lastRightTrigger = rightTriggerPressed;
            lastLeftTrigger = leftTriggerPressed;

            // Set shooter motor velocity
            shooterMotor.setVelocity(targetRPM);

            // --- Limelight Distance Calculation ---
            LLResult result = limelight.getLatestResult();
            double distance = -1; // Default value if no target is seen

            if (result.isValid()) {
                double ty = result.getTy();
                distance = (TARGET_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) /
                        Math.tan(Math.toRadians(CAMERA_MOUNT_ANGLE_DEGREES + ty));
            }

            // --- Telemetry ---
            telemetry.addLine("--- Shooter ---");
            telemetry.addData("Target RPM", "%.0f", targetRPM);
            telemetry.addData("Actual Velocity", "%.0f", shooterMotor.getVelocity());
            telemetry.addLine("\n--- Intake ---");
            telemetry.addData("Velocity", intakeMotor.getVelocity());
            telemetry.addLine("\n--- Limelight ---");
            if (distance != -1) {
                telemetry.addData("Distance (in)", "%.2f", distance);
            } else {
                telemetry.addData("Distance (in)", "No target visible");
            }
            telemetry.update();
        }

        // Stop all motors and the limelight stream when the OpMode ends
        shooterMotor.setPower(0);
        intakeMotor.setPower(0);
        limelight.stop();
    }
}

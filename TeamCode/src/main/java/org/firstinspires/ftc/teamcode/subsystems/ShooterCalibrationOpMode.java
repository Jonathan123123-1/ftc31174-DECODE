package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.ShooterConstants.*;

/**
 * Calibration OpMode for the shooter system
 * Use this to measure and tune your shooter constants
 */
@TeleOp(name="Shooter Calibration", group="Calibration")
public class ShooterCalibrationOpMode extends LinearOpMode {

    private DcMotorEx turretMotor;
    private DcMotorEx flywheelMotor;
    private Servo hoodServo;

    private enum CalibrationMode {
        HOOD_CALIBRATION,
        FLYWHEEL_CALIBRATION,
        TURRET_CALIBRATION,
        DISTANCE_MEASUREMENT
    }

    private CalibrationMode currentMode = CalibrationMode.HOOD_CALIBRATION;

    // Calibration state
    private double hoodPosition = 0.6;
    private double flywheelPower = 0.0;
    private double turretPower = 0.0;

    @Override
    public void runOpMode() {
        // Initialize hardware
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        hoodServo = hardwareMap.get(Servo.class, "turret_servo");

        telemetry.addLine("Shooter Calibration OpMode");
        telemetry.addLine("Press A to cycle through calibration modes");
        telemetry.addLine("Current mode: " + currentMode);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Switch calibration modes
            if (gamepad1.a) {
                cycleCalibrationMode();
                sleep(200); // Debounce
            }

            switch (currentMode) {
                case HOOD_CALIBRATION:
                    runHoodCalibration();
                    break;
                case FLYWHEEL_CALIBRATION:
                    runFlywheelCalibration();
                    break;
                case TURRET_CALIBRATION:
                    runTurretCalibration();
                    break;
                case DISTANCE_MEASUREMENT:
                    runDistanceMeasurement();
                    break;
            }

            telemetry.update();
        }

        // Stop everything
        turretMotor.setPower(0);
        flywheelMotor.setPower(0);
    }

    /**
     * HOOD CALIBRATION
     * Verify the servo positions for 0-22 degree range
     */
    private void runHoodCalibration() {
        telemetry.addLine("=== HOOD CALIBRATION ===");
        telemetry.addLine();
        telemetry.addLine("Use DPAD UP/DOWN to adjust hood position");
        telemetry.addLine("Current range: 0.6 (0°) to 0.8 (22°)");
        telemetry.addLine();

        // Adjust hood position
        if (gamepad1.dpad_up) {
            hoodPosition += 0.001;
            hoodPosition = Math.min(1.0, hoodPosition);
        } else if (gamepad1.dpad_down) {
            hoodPosition -= 0.001;
            hoodPosition = Math.max(0.0, hoodPosition);
        }

        hoodServo.setPosition(hoodPosition);

        // Calculate what angle this should be
        double expectedAngle = ((hoodPosition - 0.6) / 0.2) * 22.0;

        telemetry.addData("Hood Servo Position", "%.3f", hoodPosition);
        telemetry.addData("Expected Angle", "%.1f°", expectedAngle);
        telemetry.addLine();
        telemetry.addLine("INSTRUCTIONS:");
        telemetry.addLine("1. Move to position 0.6 - should be 0° (flat)");
        telemetry.addLine("2. Move to position 0.8 - should be 22° (steep)");
        telemetry.addLine("3. Measure with protractor to verify");
        telemetry.addLine("4. If off, adjust MIN/MAX values in Constants");
    }

    /**
     * FLYWHEEL CALIBRATION
     * Test the velocity multiplier (currently 72.0)
     */
    private void runFlywheelCalibration() {
        telemetry.addLine("=== FLYWHEEL CALIBRATION ===");
        telemetry.addLine();
        telemetry.addLine("Use RIGHT STICK Y to control flywheel power");
        telemetry.addLine();

        // Control flywheel with right stick
        flywheelPower = -gamepad1.right_stick_y;
        flywheelPower = Math.max(0.0, Math.min(1.0, flywheelPower));
        flywheelMotor.setPower(flywheelPower);

        // Calculate ticks/sec and RPM
        double ticksPerSec = flywheelMotor.getVelocity();
        double rpm = (ticksPerSec / ShooterConstants.FLYWHEEL_TICKS_PER_REV) * 60.0;

        // Estimate launch velocity using current multiplier
        double estimatedVelocity = ticksPerSec / ShooterConstants.FLYWHEEL_VELOCITY_MULTIPLIER;

        telemetry.addData("Flywheel Power", "%.2f", flywheelPower);
        telemetry.addData("Flywheel RPM", "%.0f", rpm);
        telemetry.addData("Ticks/Sec", "%.0f", ticksPerSec);
        telemetry.addData("Est. Launch Vel", "%.0f in/s", estimatedVelocity);
        telemetry.addData("Hood Position", "%.3f", hoodServo.getPosition());
        telemetry.addLine();
        telemetry.addLine("VERIFICATION:");
        telemetry.addLine("Your existing calibration uses multiplier 72.0");
        telemetry.addLine("This matches your lookup table data well");
        telemetry.addLine("Only change if you find inconsistencies");
    }

    /**
     * TURRET CALIBRATION
     * Test turret range of motion (±50°)
     */
    private void runTurretCalibration() {
        telemetry.addLine("=== TURRET CALIBRATION ===");
        telemetry.addLine();
        telemetry.addLine("Use LEFT STICK X to control turret");
        telemetry.addLine("Use B to reset encoder");
        telemetry.addLine();

        // Control turret with left stick
        turretPower = gamepad1.left_stick_x * 0.3; // Reduced power for safety
        turretMotor.setPower(turretPower);

        // Reset encoder
        if (gamepad1.b) {
            turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            sleep(100);
        }

        int currentTicks = turretMotor.getCurrentPosition();
        double degrees = (currentTicks / ShooterConstants.TURRET_TICKS_PER_REV) * 360.0;

        telemetry.addData("Turret Power", "%.2f", turretPower);
        telemetry.addData("Turret Ticks", "%d", currentTicks);
        telemetry.addData("Turret Degrees", "%.1f°", degrees);
        telemetry.addLine();
        telemetry.addLine("INSTRUCTIONS:");
        telemetry.addLine("1. Center the turret pointing forward");
        telemetry.addLine("2. Press B to reset encoder");
        telemetry.addLine("3. Rotate to max left (~50°)");
        telemetry.addLine("4. Rotate to max right (~-50°)");
        telemetry.addLine("5. Verify range matches limits");
    }

    /**
     * DISTANCE MEASUREMENT
     * Reference for field measurements
     */
    private void runDistanceMeasurement() {
        telemetry.addLine("=== FIELD COORDINATES ===");
        telemetry.addLine();
        telemetry.addLine("Pedro Pathing coordinates:");
        telemetry.addLine("(0,0) = bottom-left");
        telemetry.addLine("(144,144) = top-right");
        telemetry.addLine();
        telemetry.addLine("Current goal positions:");
        telemetry.addData("Red Goal", "(%.0f, %.0f)",
                ShooterConstants.RED_GOAL_X, ShooterConstants.RED_GOAL_Y);
        telemetry.addData("Blue Goal", "(%.0f, %.0f)",
                ShooterConstants.BLUE_GOAL_X, ShooterConstants.BLUE_GOAL_Y);
        telemetry.addLine();
        telemetry.addLine("Heights:");
        telemetry.addData("Shooter Height", "%.1f in", ShooterConstants.SHOOTER_HEIGHT);
        telemetry.addData("Goal Height", "%.1f in", ShooterConstants.GOAL_HEIGHT);
        telemetry.addLine();
        telemetry.addLine("Tunable parameters:");
        telemetry.addData("Pass-Through Radius", "%.1f in", ShooterConstants.PASS_THROUGH_RADIUS);
        telemetry.addData("Score Angle", "%.1f°", Math.toDegrees(ShooterConstants.SCORE_ANGLE));
    }

    private void cycleCalibrationMode() {
        switch (currentMode) {
            case HOOD_CALIBRATION:
                currentMode = CalibrationMode.FLYWHEEL_CALIBRATION;
                break;
            case FLYWHEEL_CALIBRATION:
                currentMode = CalibrationMode.TURRET_CALIBRATION;
                break;
            case TURRET_CALIBRATION:
                currentMode = CalibrationMode.DISTANCE_MEASUREMENT;
                break;
            case DISTANCE_MEASUREMENT:
                currentMode = CalibrationMode.HOOD_CALIBRATION;
                break;
        }

        // Reset motors when switching modes
        turretMotor.setPower(0);
        flywheelMotor.setPower(0);
        turretPower = 0;
        flywheelPower = 0;
    }
}
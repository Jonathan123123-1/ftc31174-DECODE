package org.firstinspires.ftc.teamcode.shooterConstants;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * TurretLockSystem
 * <p>
 * This class controls the turret to lock onto a specific field position
 * using odometry data.
 */
public class TurretLockSystem {

    // ==================== HARDWARE ====================
    private final Localizer localizer;
    private final DcMotorEx turretMotor;
    private final Telemetry telemetry;

    // ==================== TARGET CONFIGURATION ====================
    // Set this to your goal/basket position on the field (in inches)
    private Pose targetPosition = new Pose(15.0, 135.0, 0);

    // ==================== STATE ====================
    private boolean isLocked = false;

    // ==================== TURRET CONTROL ====================
    // Constants for turret motor control. You will need to tune these values.
    private static final double TURRET_TICKS_PER_DEGREE = 14.8; // EXAMPLE: Tune this for your turret
    private static final double TURRET_kP = 0.04;               // Proportional gain for alignment
    private static final double MAX_TURRET_POWER = 0.8;         // Max power for the turret

    // ==================== CONSTRUCTOR ====================
    public TurretLockSystem(
            Localizer localizer,
            DcMotorEx turretMotor,
            Telemetry telemetry
    ) {
        this.localizer = localizer;
        this.turretMotor = turretMotor;
        this.telemetry = telemetry;

        // Crucial: Set up the motor for encoder-based control
        this.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // ==================== TARGET SETUP ====================

    public void setTargetPosition(double x, double y) {
        this.targetPosition = new Pose(x, y, 0);
    }

    // ==================== LOCK ON/OFF ====================

    public void lockOn() {
        if (!isLocked) {
            isLocked = true;
            telemetry.addLine("🎯 TURRET LOCKED ON TARGET");
        }
    }

    public void unlock() {
        if (isLocked) {
            isLocked = false;
            // Return turret to zero position
            turretMotor.setTargetPosition(0);
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turretMotor.setPower(MAX_TURRET_POWER);
            telemetry.addLine("🔓 TURRET UNLOCKED, RETURNING TO ZERO");
        }
    }

    public void toggleLock() {
        if (isLocked) {
            unlock();
        } else {
            lockOn();
        }
    }

    public boolean isLocked() {
        return isLocked;
    }

    // ==================== UPDATE LOOP ====================

    /**
     * Call this in your OpMode's main loop.
     * Handles aiming the turret if locked, or returning to zero if unlocked.
     */
    public void update() {
        if (!isLocked) {
            // If not locked, let the motor finish returning to zero if it's busy.
            if (turretMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION && !turretMotor.isBusy()) {
                // Finished returning to zero
                turretMotor.setPower(0);
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            return;
        }

        // Ensure motor is in the correct mode for P-control
        if (turretMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // 1. Get current robot pose
        Pose currentPose = localizer.getPose();

        // 2. Calculate the world-frame angle from robot to target
        double dx = targetPosition.getX() - currentPose.getX();
        double dy = targetPosition.getY() - currentPose.getY();
        double absoluteAngleToTarget = Math.toDegrees(Math.atan2(dy, dx));

        // 3. Get robot's current heading
        double robotHeading = Math.toDegrees(currentPose.getHeading());

        // 4. Calculate the target angle for the turret relative to the robot's front
        double targetTurretAngle = normalizeAngle(absoluteAngleToTarget - robotHeading);

        // 5. Get the turret's current angle from its encoder
        double currentTurretAngle = turretMotor.getCurrentPosition() / TURRET_TICKS_PER_DEGREE;

        // 6. Calculate error and apply power using a P-controller
        double angleError = normalizeAngle(targetTurretAngle - currentTurretAngle);
        double power = angleError * TURRET_kP;
        power = Math.max(-MAX_TURRET_POWER, Math.min(MAX_TURRET_POWER, power));
        turretMotor.setPower(power);

        displayTelemetry(currentPose, targetTurretAngle, currentTurretAngle);
    }

    /**
     * Normalizes an angle to the range [-180, 180].
     */
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    // ==================== TELEMETRY ====================
    private void displayTelemetry(Pose currentPose, double targetAngle, double currentAngle) {
        telemetry.addData("═══ TURRET LOCK ═══", "");
        telemetry.addData("Status", isLocked ? "🎯 LOCKED" : "🔓 UNLOCKED");
        if (isLocked) {
            telemetry.addData("Robot Pose", "X: %.1f, Y: %.1f, H: %.1f", currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
            telemetry.addData("Target Angle", "%.1f deg", targetAngle);
            telemetry.addData("Current Angle", "%.1f deg", currentAngle);
            telemetry.addData("Angle Error", "%.1f deg", normalizeAngle(targetAngle - currentAngle));
            telemetry.addData("Turret Power", "%.2f", turretMotor.getPower());
        }
    }
}

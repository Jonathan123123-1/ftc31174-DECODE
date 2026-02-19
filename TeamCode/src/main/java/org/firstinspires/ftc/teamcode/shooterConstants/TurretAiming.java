package org.firstinspires.ftc.teamcode.shooterConstants;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * TurretAiming
 *
 * Simple turret control that aims at the goal using:
 * 1. Pinpoint odometry (primary) - knows where robot is, calculates angle to goal
 * 2. Limelight (optional backup) - corrects if odometry drifts
 *
 * STEP 1: Just get the turret pointing at the goal
 */
public class TurretAiming {

    // ==================== HARDWARE ====================

    private final DcMotorEx turretMotor;
    private final Limelight3A limelight;

    // ==================== GOAL POSITIONS ====================

    /** RED alliance goal (x, y in inches) - TODO: SET THESE! */
    private static final Pose RED_GOAL = new Pose(135.0, 135.0, 0);

    /** BLUE alliance goal (x, y in inches) - TODO: SET THESE! */
    private static final Pose BLUE_GOAL = new Pose(15.0, 135.0, 0);

    private Alliance currentAlliance = Alliance.BLUE;

    public enum Alliance {
        RED, BLUE
    }

    // ==================== TURRET CONSTRAINTS ====================
    // TODO: MEASURE AND SET THESE BASED ON YOUR PHYSICAL LIMITS!

    /** Minimum turret angle (radians) - can't go further counter-clockwise */
    private static final double TURRET_MIN_ANGLE_RAD = Math.toRadians(-135);

    /** Maximum turret angle (radians) - can't go further clockwise */
    private static final double TURRET_MAX_ANGLE_RAD = Math.toRadians(135);

    /** Turret home position (radians, typically 0 = straight forward) */
    private static final double TURRET_HOME_ANGLE_RAD = 0.0;

    // ==================== TURRET MOTOR CALIBRATION ====================
    // TODO: CALIBRATE THIS!
    // To calibrate: Manually rotate turret exactly 90 degrees (π/2 radians)
    // Count encoder ticks, then: TICKS_PER_RADIAN = ticks / (π/2)

    /** How many encoder ticks per radian of turret rotation */
    private static final double TURRET_TICKS_PER_RADIAN = -100.0;

    // ==================== CONTROL CONSTANTS ====================

    /** Power when moving turret to target */
    private static final double TURRET_MOVE_POWER = 0.6;

    /** Minimum power to hold position (prevent drift) */
    private static final double TURRET_HOLD_POWER = 0.05;

    /** Proportional gain for position holding */
    private static final double TURRET_HOLD_KP = 0.02;

    /** How close to target counts as "at target" (ticks) */
    private static final int TURRET_TOLERANCE_TICKS = 10;

    /** Limelight proportional gain for fine adjustment */
    private static final double LIMELIGHT_KP = 0.008;

    /** Minimum power when using Limelight */
    private static final double LIMELIGHT_MIN_POWER = 0.085;

    // ==================== STATE ====================

    private boolean isActive = false;           // Is turret aiming active?
    private boolean useLimelightCorrection = false;  // Should we use Limelight?

    private double targetAngleFieldRelative = 0.0;  // Target angle (field coords)
    private int targetTicks = 0;                     // Target in motor ticks

    // ==================== CONSTRUCTOR ====================

    /**
     * Create turret aiming controller
     *
     * @param turretMotor The turret rotation motor
     * @param limelight The Limelight camera (can be null if not using)
     */
    public TurretAiming(DcMotorEx turretMotor, Limelight3A limelight) {
        this.turretMotor = turretMotor;
        this.limelight = limelight;
    }

    // ==================== MAIN UPDATE METHOD ====================

    /**
     * Call this every loop!
     *
     * @param robotPose Current robot position from odometry
     * @param robotHeading Current robot heading (radians)
     */
    public void update(Pose robotPose, double robotHeading) {
        if (!isActive) {
            // Not aiming - stop motor
            turretMotor.setPower(0);
            return;
        }

        // Calculate angle to goal using odometry
        Pose goal = getCurrentGoal();
        double dx = goal.getX() - robotPose.getX();
        double dy = goal.getY() - robotPose.getY();
        double angleToGoal = Math.atan2(dy, dx);  // Field-relative angle

        // Clamp to turret limits
        targetAngleFieldRelative = clampAngle(angleToGoal);

        // Convert to robot-relative angle (subtract robot heading)
        double robotRelativeAngle = targetAngleFieldRelative - robotHeading;

        // Normalize to [-π, π]
        robotRelativeAngle = normalizeAngle(robotRelativeAngle);

        // Convert to motor ticks
        targetTicks = (int)(robotRelativeAngle * TURRET_TICKS_PER_RADIAN);

        // Control turret motor
        if (useLimelightCorrection && limelight != null) {
            // Use Limelight for fine adjustment
            updateWithLimelight();
        } else {
            // Use odometry only
            updateWithOdometry();
        }
    }

    // ==================== ODOMETRY-BASED CONTROL ====================

    /**
     * Control turret using odometry only (primary method)
     */
    private void updateWithOdometry() {
        int currentTicks = turretMotor.getCurrentPosition();
        int error = targetTicks - currentTicks;

        // Check if at target
        if (Math.abs(error) <= TURRET_TOLERANCE_TICKS) {
            // At target - apply small holding power to prevent drift
            double holdPower = error * TURRET_HOLD_KP;
            holdPower = Math.max(-TURRET_HOLD_POWER, Math.min(TURRET_HOLD_POWER, holdPower));
            turretMotor.setPower(holdPower);
            return;
        }

        // Not at target - move toward it
        double power = TURRET_MOVE_POWER;

        // Slow down as we approach target (proportional control)
        if (Math.abs(error) < 100) {
            power = 0.3 + (Math.abs(error) / 100.0) * (TURRET_MOVE_POWER - 0.3);
        }

        // Apply power in correct direction
        turretMotor.setPower(error > 0 ? power : -power);
    }

    // ==================== LIMELIGHT-BASED CORRECTION ====================

    /**
     * Use Limelight for fine adjustment (when odometry might have drifted)
     */
    private void updateWithLimelight() {
        LLResult result = limelight.getLatestResult();

        if (!result.isValid()) {
            // Can't see target - fall back to odometry
            updateWithOdometry();
            return;
        }

        double tx = result.getTx();  // Horizontal offset in degrees

        // If reasonably close to center, use Limelight fine control
        if (Math.abs(tx) > 3.0) {
            // Limelight proportional control
            double power = -tx * LIMELIGHT_KP + Math.signum(-tx) * LIMELIGHT_MIN_POWER;
            turretMotor.setPower(power);
        } else {
            // Very close - stop
            turretMotor.setPower(0);
        }
    }

    // ==================== CONTROL METHODS ====================

    /**
     * Start aiming at goal
     * Call this when you press the button!
     */
    public void startAiming() {
        isActive = true;
    }

    /**
     * Stop aiming and turn off motor
     */
    public void stopAiming() {
        isActive = false;
        turretMotor.setPower(0);
    }

    /**
     * Toggle aiming on/off
     */
    public void toggleAiming() {
        if (isActive) {
            stopAiming();
        } else {
            startAiming();
        }
    }

    /**
     * Enable Limelight correction
     * Call this when you press the Limelight button!
     */
    public void enableLimelightCorrection() {
        useLimelightCorrection = true;
    }

    /**
     * Disable Limelight correction (use odometry only)
     */
    public void disableLimelightCorrection() {
        useLimelightCorrection = false;
    }

    /**
     * Toggle Limelight correction on/off
     */
    public void toggleLimelightCorrection() {
        useLimelightCorrection = !useLimelightCorrection;
    }

    /**
     * Return turret to home position
     */
    public void goHome() {
        isActive = true;
        targetAngleFieldRelative = TURRET_HOME_ANGLE_RAD;
        useLimelightCorrection = false;
    }

    // ==================== ALLIANCE & GOAL ====================

    /**
     * Set which alliance you're on
     */
    public void setAlliance(Alliance alliance) {
        currentAlliance = alliance;
    }

    /**
     * Get current goal position based on alliance
     */
    public Pose getCurrentGoal() {
        return currentAlliance == Alliance.RED ? RED_GOAL : BLUE_GOAL;
    }

    // ==================== STATUS QUERIES ====================

    /**
     * Is turret currently aiming?
     */
    public boolean isAiming() {
        return isActive;
    }

    /**
     * Is turret at the target position?
     */
    public boolean isAtTarget() {
        int currentTicks = turretMotor.getCurrentPosition();
        int error = Math.abs(targetTicks - currentTicks);
        return error <= TURRET_TOLERANCE_TICKS;
    }

    /**
     * Is Limelight correction enabled?
     */
    public boolean isUsingLimelight() {
        return useLimelightCorrection;
    }

    /**
     * Get current target angle (field-relative, radians)
     */
    public double getTargetAngle() {
        return targetAngleFieldRelative;
    }

    /**
     * Get current target in motor ticks
     */
    public int getTargetTicks() {
        return targetTicks;
    }

    /**
     * Get current motor position in ticks
     */
    public int getCurrentTicks() {
        return turretMotor.getCurrentPosition();
    }

    /**
     * Get error from target (ticks)
     */
    public int getError() {
        return targetTicks - turretMotor.getCurrentPosition();
    }

    /**
     * Is target angle within safe limits?
     */
    public boolean isTargetAngleSafe() {
        return targetAngleFieldRelative >= TURRET_MIN_ANGLE_RAD &&
                targetAngleFieldRelative <= TURRET_MAX_ANGLE_RAD;
    }

    // ==================== UTILITY METHODS ====================

    /**
     * Clamp angle to turret physical limits
     */
    private double clampAngle(double angle) {
        angle = normalizeAngle(angle);

        if (angle < TURRET_MIN_ANGLE_RAD) {
            return TURRET_MIN_ANGLE_RAD;
        }
        if (angle > TURRET_MAX_ANGLE_RAD) {
            return TURRET_MAX_ANGLE_RAD;
        }
        return angle;
    }

    /**
     * Normalize angle to [-π, π]
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // ==================== TELEMETRY HELPERS ====================

    /**
     * Get status string for telemetry
     */
    public String getStatusString() {
        if (!isActive) return "INACTIVE";
        if (isAtTarget()) return "AT TARGET";
        return "MOVING";
    }

    /**
     * Get control method string for telemetry
     */
    public String getControlMethodString() {
        if (!isActive) return "N/A";
        if (useLimelightCorrection) return "LIMELIGHT";
        return "ODOMETRY";
    }
}
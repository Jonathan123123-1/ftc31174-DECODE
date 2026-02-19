package org.firstinspires.ftc.teamcode.shooterConstants;

import com.pedropathing.geometry.Pose;

/**
 * ShooterManager
 *
 * Unified interface for shooting system.
 * Intelligently chooses between:
 * - Lookup Table (proven, stationary)
 * - Physics (moving, untested positions)
 *
 * This is your main interface - use this class from your OpModes!
 */
public class ShooterManager {

    // ==================== SHOOTER MODES ====================

    public enum ShooterMode {
        AUTO,           // Automatically choose best method
        LOOKUP_TABLE,   // Force lookup table
        PHYSICS         // Force physics calculations
    }

    private static ShooterMode currentMode = ShooterMode.AUTO;

    // ==================== AUTOMATIC SELECTION THRESHOLDS ====================

    /**
     * Velocity threshold for switching to physics (in/s)
     * Above this speed, physics is used (for compensation)
     * Below this speed, lookup table is used (faster, proven)
     */
    private static final double VELOCITY_THRESHOLD = 10.0; // 10 inches per second

    /**
     * Distance threshold for using lookup table (inches)
     * Within this range, lookup table is preferred (it's tested)
     * Outside this range, physics is used
     */
    private static final double LOOKUP_TABLE_MIN_DISTANCE = 55.0;
    private static final double LOOKUP_TABLE_MAX_DISTANCE = 162.0;

    // ==================== MAIN CALCULATION METHOD ====================

    /**
     * Calculate shooter parameters - automatically chooses best method
     *
     * @param robotPose Current robot position from odometry
     * @param robotVelocity Current robot velocity from odometry
     * @return ShooterParameters with RPM, hood, turret angle
     */
    public static LookupTableShooter.ShooterParameters calculate(
            Pose robotPose, Pose robotVelocity) {

        if (currentMode == ShooterMode.LOOKUP_TABLE) {
            return LookupTableShooter.calculate(robotPose);
        }

        if (currentMode == ShooterMode.PHYSICS) {
            return PhysicsShooter.calculate(robotPose, robotVelocity);
        }

        // AUTO mode - choose intelligently
        return chooseAndCalculate(robotPose, robotVelocity);
    }

    /**
     * Automatically choose between lookup table and physics
     */
    private static LookupTableShooter.ShooterParameters chooseAndCalculate(
            Pose robotPose, Pose robotVelocity) {

        // Calculate velocity magnitude
        double velocityMagnitude = Math.sqrt(
                robotVelocity.getX() * robotVelocity.getX() +
                        robotVelocity.getY() * robotVelocity.getY()
        );

        // Calculate distance to goal
        double distance = GoalManager.getDistanceToGoal(robotPose);

        // Decision logic:
        // 1. If moving fast → Use physics (need velocity compensation)
        if (velocityMagnitude > VELOCITY_THRESHOLD) {
            return PhysicsShooter.calculate(robotPose, robotVelocity);
        }

        // 2. If distance is outside tested range → Use physics
        if (distance < LOOKUP_TABLE_MIN_DISTANCE || distance > LOOKUP_TABLE_MAX_DISTANCE) {
            return PhysicsShooter.calculate(robotPose, robotVelocity);
        }

        // 3. Otherwise → Use lookup table (proven, fast)
        return LookupTableShooter.calculate(robotPose);
    }

    // ==================== MODE CONTROL ====================

    /**
     * Set shooter mode
     */
    public static void setMode(ShooterMode mode) {
        currentMode = mode;
    }

    /**
     * Get current mode
     */
    public static ShooterMode getMode() {
        return currentMode;
    }

    /**
     * Force lookup table mode (stationary shots)
     */
    public static void useLookupTable() {
        currentMode = ShooterMode.LOOKUP_TABLE;
    }

    /**
     * Force physics mode (moving shots)
     */
    public static void usePhysics() {
        currentMode = ShooterMode.PHYSICS;
    }

    /**
     * Enable automatic mode selection (recommended)
     */
    public static void useAutoMode() {
        currentMode = ShooterMode.AUTO;
    }

    // ==================== CONVENIENCE METHODS ====================

    /**
     * Get which method would be used for current state
     * Useful for telemetry
     */
    public static String getActiveMethod(Pose robotPose, Pose robotVelocity) {
        if (currentMode != ShooterMode.AUTO) {
            return currentMode.toString();
        }

        double velocityMagnitude = Math.sqrt(
                robotVelocity.getX() * robotVelocity.getX() +
                        robotVelocity.getY() * robotVelocity.getY()
        );
        double distance = GoalManager.getDistanceToGoal(robotPose);

        if (velocityMagnitude > VELOCITY_THRESHOLD) {
            return "PHYSICS (Moving)";
        }
        if (distance < LOOKUP_TABLE_MIN_DISTANCE || distance > LOOKUP_TABLE_MAX_DISTANCE) {
            return "PHYSICS (Distance)";
        }
        return "LOOKUP_TABLE";
    }

    /**
     * Get current velocity threshold
     */
    public static double getVelocityThreshold() {
        return VELOCITY_THRESHOLD;
    }
}
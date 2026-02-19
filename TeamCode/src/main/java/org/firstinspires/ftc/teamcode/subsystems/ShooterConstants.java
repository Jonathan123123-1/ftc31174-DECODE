package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Constants for the shooter subsystem
 * Physics-based shooting with Pedro Pathing integration
 * Field coordinates: (0,0) at bottom-left, (144,144) at top-right
 */
public class ShooterConstants {

    // ==================== PHYSICAL MEASUREMENTS ====================

    /** Gravity constant in inches per second squared */
    public static double GRAVITY = 386.1; // in/s^2

    /** Height of the shooter exit point above ground */
    public static double SHOOTER_HEIGHT = 12.0; // inches - FROM YOUR CODE

    /** Height of the goal scoring zone above ground */
    public static double GOAL_HEIGHT = 29.5; // inches - FROM YOUR CODE

    /** Radial offset from goal center to pass-through point */
    public static double PASS_THROUGH_RADIUS = 28.0; // inches - FROM YOUR CODE

    /** Height adjustment above goal for pass-through point */
    public static double SCORE_HEIGHT_ADJUST = 5.0; // inches - FROM YOUR CODE

    /** Height of pass-through point above shooter */
    public static double SCORE_HEIGHT = (GOAL_HEIGHT - SHOOTER_HEIGHT) + SCORE_HEIGHT_ADJUST;

    /** Desired angle of ball when passing through point (negative = downward) */
    public static double SCORE_ANGLE = Math.toRadians(-12.0); // FROM YOUR CODE


    // ==================== HARDWARE LIMITS ====================

    /** Minimum achievable hood angle in radians (from your 0-22 degree range) */
    public static double MIN_HOOD_ANGLE = Math.toRadians(132.0);

    /** Maximum achievable hood angle in radians */
    public static double MAX_HOOD_ANGLE = Math.toRadians(118.0);

    /** Servo position for minimum hood angle (from your code) */
    public static double MIN_SERVO_POSITION = 0.191;

    /** Servo position for maximum hood angle (from your code) */
    public static double MAX_SERVO_POSITION = 0.355;

    /** Maximum turret rotation in degrees (both directions) */
    public static double MAX_TURRET_ROTATION = 456.0; // degrees

    /** Flywheel motor max RPM (from your code ~1570 RPM) */
    public static double MAX_FLYWHEEL_RPM = 1560.0;


    // ==================== MOTOR CONFIGURATION ====================

    /** Turret motor ticks per revolution (goBILDA 312 RPM motor) */
    public static double TURRET_TICKS_PER_REV = 537.7;

    /** Turret gear ratio (motor rotations : turret rotations) */
    public static double TURRET_GEAR_RATIO = 1.0;

    /** Flywheel motor ticks per revolution (goBILDA 6000 RPM motor) */
    public static double FLYWHEEL_TICKS_PER_REV = 103.8;


    // ==================== CALIBRATION DATA ====================

    /**
     * Flywheel velocity conversion (from your code: 72.0 multiplier)
     * flywheel_ticks_per_sec = FLYWHEEL_VELOCITY_MULTIPLIER * launch_velocity_in_per_sec
     */
    public static double FLYWHEEL_VELOCITY_MULTIPLIER = 0.2; // FROM YOUR CODE

    /**
     * Your existing lookup table data for reference
     * Can be used for validation or fallback
     */
    public static class LookupTableData {
        public static final double DIST_CLOSEST = 48.0;
        public static final double DIST_MID_CLOSE = 56.0;
        public static final double DIST_MID_FAR = 96.0;
        public static final double DIST_SECOND_FAR = 137.5;
        public static final double DIST_FAR = 144.0;

        public static final double RPM_CLOSEST = 1200;
        public static final double RPM_MID_CLOSE = 1250;
        public static final double RPM_MID_FAR = 1380;
        public static final double RPM_SECOND_FAR = 1560;
        public static final double RPM_FAR = 1560;
    }


    // ==================== GOAL POSITIONS (PEDRO PATHING COORDINATES) ====================

    /** Red alliance goal position (x, y) - Bottom-left quadrant */
    public static double RED_GOAL_X = 135.0; // FROM YOUR CODE
    public static double RED_GOAL_Y = 135.0; // FROM YOUR CODE

    /** Blue alliance goal position (x, y) - Top-right quadrant */
    public static double BLUE_GOAL_X = 15.0; // FROM YOUR CODE
    public static double BLUE_GOAL_Y = 135.0; // FROM YOUR CODE


    // ==================== TUNING PARAMETERS ====================

    /** Turret PID constants - Using your existing values */
    public static double TURRET_KP = 0.05;
    public static double TURRET_KI = 0.0;
    public static double TURRET_KD = 0.002;

    /** Flywheel PID constants - Adapted from your kP and kF values */
    public static double FLYWHEEL_KP = 75.0; // Your average kP value
    public static double FLYWHEEL_KI = 0.0;
    public static double FLYWHEEL_KD = 0.0;
    public static double FLYWHEEL_KF = 11.7; // Your kF value

    /** Tolerance for shooter ready state */
    public static double FLYWHEEL_RPM_TOLERANCE = 50.0; // RPM
    public static double TURRET_ANGLE_TOLERANCE = Math.toRadians(2.0); // radians
    public static double HOOD_POSITION_TOLERANCE = 0.02; // servo position


    // ==================== SAFETY ====================

    /** Maximum allowable launch velocity */
    public static double MAX_LAUNCH_VELOCITY = 300.0; // in/s

    /** Minimum distance to goal before shooting is allowed */
    public static double MIN_SHOOTING_DISTANCE = 39.0; // inches

    /** Maximum distance to goal where shooting is reliable */
    public static double MAX_SHOOTING_DISTANCE = 150.0; // inches

// ==================== LIMELIGHT CONFIGURATION ====================

    /** Limelight 3A mount height above ground (MEASURE THIS!) */
    public static double LIMELIGHT_HEIGHT = 12.0; // inches - UPDATE THIS

    /** Limelight mount angle from horizontal (MEASURE THIS!) */
    public static double LIMELIGHT_MOUNT_ANGLE = 0.0; // degrees - UPDATE THIS

    /** Maximum acceptable difference between Limelight and odometry */
    public static double LIMELIGHT_ODOMETRY_TOLERANCE = 5.0; // inches

    /** Use Limelight for verification */
    public static boolean USE_LIMELIGHT_VERIFICATION = true;

    /** Use Limelight as primary (if you don't trust odometry) */
    public static boolean USE_LIMELIGHT_PRIMARY = false;

    // ==================== HELPER METHODS ====================

    /**
     * Normalize angle to [-180, 180] degrees
     */
    public static double normalizeAngleDegrees(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }

    /**
     * Normalize angle to [-PI, PI] radians
     */
    public static double normalizeAngleRadians(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}
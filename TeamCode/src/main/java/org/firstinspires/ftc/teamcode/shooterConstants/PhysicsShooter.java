package org.firstinspires.ftc.teamcode.shooterConstants;

import com.pedropathing.geometry.Pose;

/**
 * PhysicsShooter
 *
 * Implements Gyrobotic Droids physics-based shooting with velocity compensation.
 * Calculates optimal trajectory using projectile motion equations.
 *
 * Use this for:
 * - Shooting while moving
 * - Untested field positions
 * - Maximum theoretical accuracy
 */
public class PhysicsShooter {

    // ==================== PHYSICS CONSTANTS ====================

    /** Gravity in inches per second squared */
    private static final double GRAVITY = 386.1; // in/s²

    // ==================== TUNABLE PARAMETERS (START HERE!) ====================

    /**
     * Height of the pass-through point above the ground (inches)
     * This is where you want the ball to pass through above the goal.
     *
     * TUNING: Start with goal rim height
     * - Too low shots → Increase this
     * - Too high shots → Decrease this
     */
    private static final double PASS_THROUGH_HEIGHT = 31.5;

    /**
     * Radial distance from goal center to pass-through point (inches)
     * Positive = point is between robot and goal
     *
     * TUNING: Start with 8-10 inches
     * - Hits front rim → Increase (moves point toward robot)
     * - Hits back wall → Decrease (moves point toward goal)
     */
    private static final double PASS_THROUGH_RADIUS = 8.0;

    /**
     * Angle of ball trajectory at pass-through point (degrees, NEGATIVE)
     * This is the angle the ball is traveling when it passes through the point.
     *
     * TUNING: Start with -30 degrees
     * - More negative = steeper descent (better for backspin)
     * - Less negative = flatter arc (more range)
     */
    private static final double PASS_THROUGH_ANGLE_DEG = -30.0;

    /**
     * Height of your launcher above the ground (inches)
     */
    private static final double LAUNCHER_HEIGHT = 12.0;

    /**
     * Minimum hood angle your mechanism can achieve (degrees)
     */
    private static final double MIN_HOOD_ANGLE = 40.0;

    /**
     * Maximum hood angle your mechanism can achieve (degrees)
     */
    private static final double MAX_HOOD_ANGLE = 60.0;

    /**
     * Minimum velocity threshold for compensation (in/s)
     * Below this speed, don't bother compensating
     */
    private static final double MIN_VELOCITY_FOR_COMPENSATION = 2.0;

    // ==================== CALCULATION METHOD ====================

    /**
     * Calculate shooter parameters using physics
     *
     * @param robotPose Current robot position
     * @param robotVelocity Robot velocity pose (from follower.getVelocity())
     * @return ShooterParameters with all calculated values
     */
    public static LookupTableShooter.ShooterParameters calculate(
            Pose robotPose, Pose robotVelocity) {

        try {
            // Get goal position
            Pose goal = GoalManager.getCurrentGoal();

            // Calculate vector from robot to goal
            double dx = goal.getX() - robotPose.getX();
            double dy = goal.getY() - robotPose.getY();
            double distanceToGoal = Math.sqrt(dx * dx + dy * dy);
            double angleToGoal = Math.atan2(dy, dx);

            // Distance to pass-through point
            double distanceToPassThrough = distanceToGoal - PASS_THROUGH_RADIUS;

            // Height difference (pass-through relative to launcher)
            double heightDiff = PASS_THROUGH_HEIGHT - LAUNCHER_HEIGHT;

            // ===== STAGE A: Calculate Initial Parameters (Stationary) =====

            double thetaRad = Math.toRadians(PASS_THROUGH_ANGLE_DEG);
            double x = distanceToPassThrough;
            double y = heightDiff;

            // Launch angle: α = arctan(2y/x - tan(θ))
            double launchAngleRad = Math.atan(2.0 * y / x - Math.tan(thetaRad));
            double launchAngleDeg = Math.toDegrees(launchAngleRad);

            // Clamp to mechanical limits
            launchAngleDeg = Math.max(MIN_HOOD_ANGLE, Math.min(MAX_HOOD_ANGLE, launchAngleDeg));
            launchAngleRad = Math.toRadians(launchAngleDeg);

            // Launch velocity: v₀ = √[gx² / (2cos²(α)(x·tan(α) - y))]
            double cosAlpha = Math.cos(launchAngleRad);
            double tanAlpha = Math.tan(launchAngleRad);
            double denominator = 2.0 * cosAlpha * cosAlpha * (x * tanAlpha - y);

            if (denominator <= 0) {
                // Invalid trajectory
                return createInvalidResult();
            }

            double v0Squared = (GRAVITY * x * x) / denominator;
            double launchVelocity = Math.sqrt(v0Squared);

            // ===== STAGE B: Velocity Compensation (if moving) =====

            double velocityMagnitude = Math.sqrt(
                    robotVelocity.getX() * robotVelocity.getX() +
                            robotVelocity.getY() * robotVelocity.getY()
            );

            double velocityOffset = 0.0;

            if (velocityMagnitude > MIN_VELOCITY_FOR_COMPENSATION) {
                // Robot is moving - apply compensation

                // 1. Break velocity into radial and tangential components
                double velocityAngle = Math.atan2(robotVelocity.getY(), robotVelocity.getX());
                double deltaTheta = velocityAngle - angleToGoal;

                double radialVelocity = velocityMagnitude * Math.cos(deltaTheta);
                double tangentialVelocity = velocityMagnitude * Math.sin(deltaTheta);

                // 2. Calculate flight time
                double flightTime = x / (launchVelocity * cosAlpha);

                // 3. Compensate horizontal velocity
                double vxInitial = launchVelocity * cosAlpha;
                double vxIntermediate = vxInitial + radialVelocity;
                double vxNew = Math.sqrt(vxIntermediate * vxIntermediate +
                        tangentialVelocity * tangentialVelocity);

                // 4. Keep vertical velocity
                double vy = launchVelocity * Math.sin(launchAngleRad);

                // 5. Recalculate launch angle
                double newLaunchAngleRad = Math.atan(vy / vxNew);
                launchAngleDeg = Math.toDegrees(newLaunchAngleRad);

                // Clamp again
                launchAngleDeg = Math.max(MIN_HOOD_ANGLE, Math.min(MAX_HOOD_ANGLE, launchAngleDeg));
                newLaunchAngleRad = Math.toRadians(launchAngleDeg);

                // 6. Recalculate launch velocity
                double xNew = vxNew * flightTime;
                cosAlpha = Math.cos(newLaunchAngleRad);
                tanAlpha = Math.tan(newLaunchAngleRad);
                denominator = 2.0 * cosAlpha * cosAlpha * (xNew * tanAlpha - y);

                if (denominator > 0) {
                    v0Squared = (GRAVITY * xNew * xNew) / denominator;
                    launchVelocity = Math.sqrt(v0Squared);
                }

                // 7. Calculate turret angle offset
                velocityOffset = Math.atan(tangentialVelocity / vxIntermediate);
            }

            // ===== STAGE C: Convert to Motor Commands =====

            // Convert hood angle to servo position
            double hoodPosition = angleToServoPosition(launchAngleDeg);

            // Convert velocity to RPM
            double rpm = velocityToRPM(launchVelocity, launchAngleDeg);

            return new LookupTableShooter.ShooterParameters(
                    rpm,
                    hoodPosition,
                    angleToGoal,
                    distanceToGoal,
                    velocityOffset,
                    true,
                    "PHYSICS"
            );

        } catch (Exception e) {
            return createInvalidResult();
        }
    }

    // ==================== HARDWARE CONVERSION ====================

    /**
     * Convert hood angle to servo position
     * Uses linear interpolation between min/max points
     */
    private static double angleToServoPosition(double angleDeg) {
        // Servo positions from your ShooterConstants
        double servoMin = 0.157; // At MIN_HOOD_ANGLE
        double servoMax = 0.211; // At MAX_HOOD_ANGLE

        // Clamp angle
        angleDeg = Math.max(MIN_HOOD_ANGLE, Math.min(MAX_HOOD_ANGLE, angleDeg));

        // Linear interpolation
        double slope = (servoMax - servoMin) / (MAX_HOOD_ANGLE - MIN_HOOD_ANGLE);
        return servoMin + slope * (angleDeg - MIN_HOOD_ANGLE);
    }

    /**
     * Convert launch velocity to flywheel RPM
     *
     * THIS IS A SIMPLIFIED VERSION - You should calibrate this!
     * See CalibrationTeleOp for the proper calibration process.
     */
    private static double velocityToRPM(double velocityInchesPerSec, double hoodAngleDeg) {
        // Simplified linear relationship
        // TODO: Replace with calibrated data

        // Rough approximation: RPM = velocity * conversion factor
        double baseRPM = velocityInchesPerSec * 7.0; // Example factor

        // Adjust for hood angle (higher angle = more RPM needed)
        double angleFactor = 1.0 + (hoodAngleDeg - MIN_HOOD_ANGLE) / (MAX_HOOD_ANGLE - MIN_HOOD_ANGLE) * 0.2;

        return baseRPM * angleFactor;
    }

    /**
     * Create invalid result when calculations fail
     */
    private static LookupTableShooter.ShooterParameters createInvalidResult() {
        return new LookupTableShooter.ShooterParameters(
                0, 0, 0, 0, 0, false, "INVALID"
        );
    }

    // ==================== TUNING HELPERS ====================

    /**
     * Get current pass-through point parameters (for telemetry)
     */
    public static String getPassThroughInfo() {
        return String.format("Height: %.1f\", Radius: %.1f\", Angle: %.1f°",
                PASS_THROUGH_HEIGHT, PASS_THROUGH_RADIUS, PASS_THROUGH_ANGLE_DEG);
    }
}
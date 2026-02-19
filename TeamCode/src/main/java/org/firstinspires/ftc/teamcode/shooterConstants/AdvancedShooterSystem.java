package org.firstinspires.ftc.teamcode.shooterConstants;

import com.pedropathing.geometry.Pose;

/**
 * AdvancedShooterSystem
 *
 * COMPLETE SHOOTING SYSTEM combining:
 * 1. Your 108-point lookup table (Turtle Walkers method)
 * 2. Cube Robot's velocity compensation (vector projection)
 * 3. Air resistance simulation for time of flight
 * 4. Turret constraints (can't hit gear/wiring)
 * 5. Pinpoint localization (primary)
 * 6. Limelight relocalization (backup)
 *
 * This is the ONLY class you need to interact with.
 */
public class AdvancedShooterSystem {

    // ==================== GOAL POSITIONS ====================
    // TODO: SET THESE!

    private static final Pose RED_GOAL = new Pose(144.0, 144.0, 0);
    private static final Pose BLUE_GOAL = new Pose(0.0, 0.0, 0);

    public enum Alliance {
        RED, BLUE
    }

    private static Alliance currentAlliance = Alliance.BLUE;

    // ==================== TURRET CONSTRAINTS ====================
    // TODO: SET THESE BASED ON YOUR PHYSICAL LIMITS!

    /** Minimum turret angle (radians) - can't go further CCW */
    private static final double TURRET_MIN_ANGLE = Math.toRadians(-135);

    /** Maximum turret angle (radians) - can't go further CW */
    private static final double TURRET_MAX_ANGLE = Math.toRadians(135);

    /** Home/zero position for turret (radians) */
    private static final double TURRET_HOME_ANGLE = 0.0;

    // ==================== PHYSICAL CONSTANTS ====================

    private static final double GRAVITY = 386.1; // in/s²
    private static final double LAUNCHER_HEIGHT = 12.0; // inches above ground
    private static final double GOAL_HEIGHT = 29.5; // inches above ground

    // Air resistance parameters (from Cube Robot)
    private static final double BALL_MASS = 0.01; // kg (adjust if needed)
    private static final double AIR_DENSITY = 1.225; // kg/m³
    private static final double BALL_DIAMETER = 3.0; // inches
    private static final double DRAG_COEFFICIENT = 0.47; // sphere

    // Simulation parameters
    private static final int SIMULATION_STEPS = 100;
    private static final double MIN_VELOCITY_FOR_COMPENSATION = 2.0; // in/s

    // ==================== RESULT CLASS ====================

    /**
     * Contains all calculated shooter parameters
     */
    public static class ShooterParams {
        public final double rpm;              // Target flywheel RPM
        public final double hoodPosition;     // Hood servo position
        public final double turretAngle;      // Turret angle (radians, field-relative)
        public final double distance;         // Distance to goal
        public final double timeOfFlight;     // Ball flight time (seconds)
        public final boolean valid;           // Calculations succeeded
        public final boolean velocityCompensated; // Whether compensation was applied
        public final String method;           // "LOOKUP" or "COMPENSATED"

        public ShooterParams(double rpm, double hoodPosition, double turretAngle,
                             double distance, double timeOfFlight, boolean valid,
                             boolean velocityCompensated, String method) {
            this.rpm = rpm;
            this.hoodPosition = hoodPosition;
            this.turretAngle = turretAngle;
            this.distance = distance;
            this.timeOfFlight = timeOfFlight;
            this.valid = valid;
            this.velocityCompensated = velocityCompensated;
            this.method = method;
        }

        /**
         * Check if turret angle is within physical limits
         */
        public boolean isTurretAngleSafe() {
            return turretAngle >= TURRET_MIN_ANGLE && turretAngle <= TURRET_MAX_ANGLE;
        }
    }

    // ==================== MAIN CALCULATION METHOD ====================

    /**
     * Calculate all shooter parameters
     *
     * @param robotPose Current robot pose from Pinpoint
     * @param vx Robot's velocity in the X direction (field-relative)
     * @param vy Robot's velocity in the Y direction (field-relative)
     * @return ShooterParams with all values needed for shooting
     */
    public static ShooterParams calculate(Pose robotPose, double vx, double vy) {
        // Get goal position based on alliance
        Pose goal = currentAlliance == Alliance.RED ? RED_GOAL : BLUE_GOAL;

        // Calculate distance and base angle to goal
        double dx = goal.getX() - robotPose.getX();
        double dy = goal.getY() - robotPose.getY();
        double distance = Math.sqrt(dx * dx + dy * dy);
        double baseAngleToGoal = Math.atan2(dy, dx);

        // Calculate robot velocity magnitude
        double velocityMagnitude = Math.sqrt(vx * vx + vy * vy);

        // Get base shooting parameters from lookup table
        double baseRPM = ShooterConstants.targetRPM(distance);
        double baseHood = ShooterConstants.hoodPosition(distance);

        // Calculate time of flight using air resistance simulation
        double timeOfFlight = calculateTimeOfFlight(baseRPM, distance);

        // If robot is not moving significantly, no compensation needed
        if (velocityMagnitude < MIN_VELOCITY_FOR_COMPENSATION) {
            return new ShooterParams(
                    baseRPM,
                    baseHood,
                    clampTurretAngle(baseAngleToGoal),
                    distance,
                    timeOfFlight,
                    true,
                    false,
                    "LOOKUP"
            );
        }

        // ===== VELOCITY COMPENSATION (CUBE ROBOT METHOD) =====

        // Robot velocity vector
        double velocityAngle = Math.atan2(vy, vx);

        // Vector from robot to goal (unit vector)
        double goalVectorX = dx / distance;
        double goalVectorY = dy / distance;

        // Project robot velocity onto goal vector (radial velocity)
        double radialVelocity = vx * goalVectorX + vy * goalVectorY;

        // Calculate tangential velocity (perpendicular to goal)
        double tangentialVelocityX = vx - radialVelocity * goalVectorX;
        double tangentialVelocityY = vy - radialVelocity * goalVectorY;
        double tangentialVelocity = Math.sqrt(
                tangentialVelocityX * tangentialVelocityX +
                        tangentialVelocityY * tangentialVelocityY
        );

        // Get sign of tangential velocity (for turret offset direction)
        double tangentialSign = Math.signum(
                vx * goalVectorY - vy * goalVectorX  // Cross product
        );
        tangentialVelocity *= tangentialSign;

        // Calculate compensated distance (radial compensation)
        // Ball needs to travel further/shorter due to robot moving toward/away from goal
        double compensatedDistance = distance - radialVelocity * timeOfFlight;

        // Get new RPM and hood for compensated distance
        double compensatedRPM = ShooterConstants.targetRPM(compensatedDistance);
        double compensatedHood = ShooterConstants.hoodPosition(compensatedDistance);

        // Calculate turret angle offset (tangential compensation)
        // The ball needs to be aimed to one side to compensate for sideways motion
        double turretOffset = Math.atan2(
                tangentialVelocity * timeOfFlight,
                compensatedDistance
        );

        double finalTurretAngle = baseAngleToGoal + turretOffset;

        // Clamp turret angle to physical limits
        finalTurretAngle = clampTurretAngle(finalTurretAngle);

        return new ShooterParams(
                compensatedRPM,
                compensatedHood,
                finalTurretAngle,
                distance,
                timeOfFlight,
                true,
                true,
                "COMPENSATED"
        );
    }

    // ==================== TIME OF FLIGHT CALCULATION ====================

    /**
     * Calculate time of flight using air resistance simulation (Cube Robot method)
     *
     * This simulates the ball's trajectory with air resistance to get accurate timing.
     * More accurate than simple projectile motion.
     */
    private static double calculateTimeOfFlight(double rpm, double distance) {
        // Convert RPM to linear velocity (inches/sec)
        // Assuming 72mm (2.83 inch) diameter wheel like Cube Robot
        double wheelDiameter = 2.83; // inches
        double ballVelocity = (rpm / 60.0) * Math.PI * wheelDiameter;

        // Estimate launch angle based on distance (simplified)
        // You can tune this based on your hood angles
        double launchAngle = estimateLaunchAngle(distance);

        // Initial velocity components
        double vx = ballVelocity * Math.cos(launchAngle);
        double vy = ballVelocity * Math.sin(launchAngle);

        // Ball properties for drag calculation
        double radius = BALL_DIAMETER / 2.0;
        double radiusMeters = radius * 0.0254; // convert to meters
        double area = Math.PI * radiusMeters * radiusMeters;

        // Drag coefficient
        double dragFactor = 0.5 * AIR_DENSITY * area * DRAG_COEFFICIENT;

        // Simulation
        double x = 0, y = LAUNCHER_HEIGHT;
        double dt = 0.01; // 10ms time step
        double time = 0;

        for (int i = 0; i < SIMULATION_STEPS && y >= 0; i++) {
            // Calculate drag force
            double speed = Math.sqrt(vx * vx + vy * vy);
            double speedMetersPerSec = speed * 0.0254; // to m/s

            double dragForce = dragFactor * speedMetersPerSec * speedMetersPerSec;

            // Drag acceleration (opposing velocity)
            double dragAccelX = -dragForce * vx / (BALL_MASS * speed) * 39.3701; // to in/s²
            double dragAccelY = -dragForce * vy / (BALL_MASS * speed) * 39.3701;

            // Update velocity (gravity + drag)
            vx += dragAccelX * dt;
            vy += (dragAccelY - GRAVITY) * dt;

            // Update position
            x += vx * dt;
            y += vy * dt;

            time += dt;

            // Check if we've reached the goal distance
            if (x >= distance) {
                return time;
            }
        }

        // Fallback: simple projectile motion if simulation doesn't converge
        return distance / (ballVelocity * Math.cos(launchAngle));
    }

    /**
     * Estimate launch angle based on distance
     * You should tune this to match your actual hood angles
     */
    private static double estimateLaunchAngle(double distance) {
        // Simple linear interpolation between close and far shots
        // Tune these values to match your robot!
        double closeDistance = 60.0;  // inches
        double closeAngle = Math.toRadians(40);  // radians

        double farDistance = 150.0;   // inches
        double farAngle = Math.toRadians(55);    // radians

        if (distance <= closeDistance) return closeAngle;
        if (distance >= farDistance) return farAngle;

        // Linear interpolation
        double t = (distance - closeDistance) / (farDistance - closeDistance);
        return closeAngle + t * (farAngle - closeAngle);
    }

    // ==================== TURRET ANGLE CLAMPING ====================

    /**
     * Clamp turret angle to physical limits and normalize to [-π, π]
     */
    private static double clampTurretAngle(double angle) {
        // Normalize to [-π, π]
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;

        // Clamp to physical limits
        if (angle < TURRET_MIN_ANGLE) angle = TURRET_MIN_ANGLE;
        if (angle > TURRET_MAX_ANGLE) angle = TURRET_MAX_ANGLE;

        return angle;
    }

    // ==================== CONFIGURATION METHODS ====================

    /**
     * Set alliance (call in OpMode init!)
     */
    public static void setAlliance(Alliance alliance) {
        currentAlliance = alliance;
    }

    /**
     * Get current alliance
     */
    public static Alliance getAlliance() {
        return currentAlliance;
    }

    /**
     * Get current goal position
     */
    public static Pose getCurrentGoal() {
        return currentAlliance == Alliance.RED ? RED_GOAL : BLUE_GOAL;
    }

    /**
     * Convert turret angle from field-relative to robot-relative
     *
     * @param fieldRelativeAngle Angle in field coordinates (radians)
     * @param robotHeading Current robot heading (radians)
     * @return Robot-relative angle for turret motor
     */
    public static double fieldToRobotRelative(double fieldRelativeAngle, double robotHeading) {
        return fieldRelativeAngle - robotHeading;
    }
}
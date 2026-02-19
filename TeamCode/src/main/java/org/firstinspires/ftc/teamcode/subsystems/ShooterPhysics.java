package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.ShooterConstants.*;

/**
 * Physics calculations for shooter trajectory
 * Based on Team 23435 methodology with your existing calibration
 */
public class ShooterPhysics {

    /**
     * Container for calculated shooter parameters
     */
    public static class ShooterSolution {
        public double launchAngle;        // radians
        public double launchVelocity;     // in/s
        public double turretAngle;        // degrees (robot-relative)
        public double flywheelTicksPerSec; // Motor ticks/sec
        public double hoodServoPosition;  // 0.0 to 1.0
        public boolean isValid;           // whether solution is achievable

        public ShooterSolution() {
            this.isValid = false;
        }
    }

    /**
     * Calculate complete shooter solution with velocity compensation
     * Integrated with Pedro Pathing coordinate system
     *
     * @param robotX robot X position (inches, 0-144)
     * @param robotY robot Y position (inches, 0-144)
     * @param robotHeading current robot heading (radians)
     * @param robotVx robot velocity in X direction (in/s)
     * @param robotVy robot velocity in Y direction (in/s)
     * @param isRedAlliance true for red alliance, false for blue
     * @return complete shooter solution
     */
    public static ShooterSolution calculateShooterSolution(
            double robotX, double robotY, double robotHeading,
            double robotVx, double robotVy, boolean isRedAlliance) {

        // Get goal position based on alliance
        double goalX = isRedAlliance ? RED_GOAL_X : BLUE_GOAL_X;
        double goalY = isRedAlliance ? RED_GOAL_Y : BLUE_GOAL_Y;

        // Calculate vector to goal
        double dx = goalX - robotX;
        double dy = goalY - robotY;
        double distToGoal = Math.sqrt(dx * dx + dy * dy);
        double angleToGoal = Math.atan2(dy, dx);

        // Adjust for pass-through point (slightly in front of goal)
        double x = distToGoal - PASS_THROUGH_RADIUS;
        double y = SCORE_HEIGHT;

        // Validate minimum distance
        if (x < MIN_SHOOTING_DISTANCE || distToGoal > MAX_SHOOTING_DISTANCE) {
            return new ShooterSolution(); // Invalid - out of range
        }

        // ========== STEP 1: CALCULATE STATIC TRAJECTORY ==========

        // Calculate theoretical hood angle using your formula
        // hoodAngle = atan((2y/x) - tan(scoreAngle))
        double hoodAngle = Math.atan((2 * y / x) - Math.tan(SCORE_ANGLE));

        // Clamp to achievable angles
        if (hoodAngle < MIN_HOOD_ANGLE || hoodAngle > MAX_HOOD_ANGLE) {
            hoodAngle = Math.max(MIN_HOOD_ANGLE, Math.min(MAX_HOOD_ANGLE, hoodAngle));
        }

        // Calculate theoretical launch velocity
        // v0 = sqrt((g * x^2) / (2 * cos^2(hood) * (x*tan(hood) - y)))
        double cosHood = Math.cos(hoodAngle);
        double tanHood = Math.tan(hoodAngle);

        double denominator = 2 * cosHood * cosHood * (x * tanHood - y);
        if (Math.abs(denominator) < 0.001) {
            return new ShooterSolution(); // Invalid trajectory
        }

        double v0Squared = (GRAVITY * x * x) / denominator;
        if (v0Squared < 0) {
            return new ShooterSolution(); // Invalid trajectory
        }

        double v0 = Math.sqrt(v0Squared);

        // ========== STEP 2: VELOCITY COMPENSATION ==========

        // Calculate robot velocity magnitude and direction
        double robotVelocityMag = Math.sqrt(robotVx * robotVx + robotVy * robotVy);
        double robotVelocityAngle = Math.atan2(robotVy, robotVx);

        // Angle difference between robot velocity and line to goal
        double relHeading = robotVelocityAngle - angleToGoal;

        // Split into parallel (radial) and perpendicular (tangential) components
        double vParallel = Math.cos(relHeading) * robotVelocityMag;
        double vPerpendicular = Math.sin(relHeading) * robotVelocityMag;

        // Calculate time for ball to reach goal
        double time = x / (v0 * cosHood);

        // Adjust for robot's radial movement
        double v0Radial = (x / time) - vParallel;

        // Calculate turret offset for perpendicular movement
        double turretOffset = Math.atan2(vPerpendicular * time, distToGoal);

        // Calculate final velocity (compensated)
        double v0Final = Math.hypot(v0Radial, vPerpendicular / cosHood);

        // Recalculate hood angle with new velocity if needed
        // For simplicity, keeping hood angle the same as it's primarily geometry-based
        // The velocity compensation handles the movement

        // ========== STEP 3: CONVERT TO HARDWARE COMMANDS ==========

        ShooterSolution solution = new ShooterSolution();
        solution.launchAngle = hoodAngle;
        solution.launchVelocity = v0Final;

        // Turret angle (robot-relative, in degrees)
        double turretAngleRad = angleToGoal - robotHeading + turretOffset;
        solution.turretAngle = Math.toDegrees(normalizeAngleRadians(turretAngleRad));

        // Check if turret can reach this angle
        if (Math.abs(solution.turretAngle) > MAX_TURRET_ROTATION) {
            solution.turretAngle = Math.max(-MAX_TURRET_ROTATION,
                    Math.min(MAX_TURRET_ROTATION, solution.turretAngle));
            solution.isValid = false; // Mark as compromised
        } else {
            solution.isValid = true;
        }

        // Convert to servo position (using your 0.6-0.8 range for 0-22 degrees)
        solution.hoodServoPosition = angleToServoPosition(hoodAngle);

        // Convert to flywheel ticks/sec (using your calibration)
        solution.flywheelTicksPerSec = velocityToFlywheelTicks(v0Final);

        // Final validation
        if (solution.flywheelTicksPerSec > MAX_FLYWHEEL_RPM * FLYWHEEL_TICKS_PER_REV / 60.0) {
            solution.isValid = false;
        }

        if (solution.launchVelocity > MAX_LAUNCH_VELOCITY) {
            solution.isValid = false;
        }

        return solution;
    }

    /**
     * Convert hood angle to servo position
     * Maps 0-22 degrees to 0.6-0.8 servo range (from your code)
     */
    public static double angleToServoPosition(double angleRadians) {
        double angleDegrees = Math.toDegrees(angleRadians);

        // Clamp to valid range
        angleDegrees = Math.max(0, Math.min(22.0, angleDegrees));

        // Linear mapping: 0° = 0.6, 22° = 0.8
        return MIN_SERVO_POSITION + (angleDegrees / 22.0) * (MAX_SERVO_POSITION - MIN_SERVO_POSITION);
    }

    /**
     * Convert launch velocity to flywheel ticks per second
     * Uses your calibration: ticks/sec = 72.0 * velocity
     */
    public static double velocityToFlywheelTicks(double launchVelocity) {
        return FLYWHEEL_VELOCITY_MULTIPLIER * launchVelocity;
    }

    /**
     * Convert flywheel ticks/sec to RPM for display
     */
    public static double ticksPerSecToRPM(double ticksPerSec) {
        return (ticksPerSec / FLYWHEEL_TICKS_PER_REV) * 60.0;
    }
}
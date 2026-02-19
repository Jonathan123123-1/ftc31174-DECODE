package org.firstinspires.ftc.teamcode.shooterConstants;

import com.pedropathing.geometry.Pose;

/**
 * LookupTableShooter
 *
 * Uses your existing 108-point lookup table for shooting.
 * This is the PROVEN, CONSISTENT method from Turtle Walkers.
 *
 * NO velocity compensation - for stationary shots only.
 * Fast, reliable, and already tested by your team.
 */
public class LookupTableShooter {

    /**
     * Calculate shooter parameters using lookup table
     *
     * @param robotPose Current robot position
     * @return ShooterParameters with RPM and hood position
     */
    public static ShooterParameters calculate(Pose robotPose) {
        // Get distance to goal
        double distance = GoalManager.getDistanceToGoal(robotPose);

        // Use your existing ShooterConstants lookup table
        double rpm = ShooterConstants.targetRPM(distance);
        double hoodPosition = ShooterConstants.hoodPosition(distance);

        // Get angle to goal for turret
        double turretAngle = GoalManager.getAngleToGoal(robotPose);

        return new ShooterParameters(
                rpm,
                hoodPosition,
                turretAngle,
                distance,
                0, // No velocity offset
                true,
                "LOOKUP_TABLE"
        );
    }

    /**
     * Container for shooter parameters
     */
    public static class ShooterParameters {
        public final double rpm;
        public final double hoodPosition;
        public final double turretAngle; // radians
        public final double distance;
        public final double velocityOffset; // radians
        public final boolean valid;
        public final String method;

        public ShooterParameters(double rpm, double hoodPosition, double turretAngle,
                                 double distance, double velocityOffset,
                                 boolean valid, String method) {
            this.rpm = rpm;
            this.hoodPosition = hoodPosition;
            this.turretAngle = turretAngle;
            this.distance = distance;
            this.velocityOffset = velocityOffset;
            this.valid = valid;
            this.method = method;
        }

        /**
         * Get total turret angle (base + velocity offset)
         */
        public double getTotalTurretAngle() {
            return turretAngle + velocityOffset;
        }
    }
}
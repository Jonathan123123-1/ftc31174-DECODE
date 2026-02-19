package org.firstinspires.ftc.teamcode.shooterConstants;

import com.pedropathing.geometry.Pose;

/**
 * distanceLocalization
 *
 * This class provides field-relative tracking for the turret.
 * It uses the robot's current pose (from Pinpoint/Odometry) to calculate
 * the exact distance and heading needed for the turret to lock onto the goal.
 */
public class distanceLocalization {

    // --- FIELD GOAL COORDINATES (Inches) ---
    // Adjust these based on your specific field orientation.
    // Assuming (0,0) is center of field, goals are typically on the back wall.
    // If your center is (72,72), adjust accordingly.
    
    // Example coordinates for the high goals:
    public static final double BLUE_GOAL_X = 20.0;
    public static final double BLUE_GOAL_Y = 112.0;

    public static final double RED_GOAL_X = -112.0;
    public static final double RED_GOAL_Y = -110.0;

    /**
     * Calculates the distance from the robot to the specified goal.
     * This distance should be passed to ShooterConstants.targetRPM()
     * 
     * @param robotPose The current pose of the robot from the follower/pinpoint.
     * @param isRedAlliance True if on Red alliance, False for Blue.
     * @return Straight-line distance to goal in inches.
     */
    public static double getDistanceToGoal(Pose robotPose, boolean isRedAlliance) {
        double targetX = isRedAlliance ? RED_GOAL_X : BLUE_GOAL_X;
        double targetY = isRedAlliance ? RED_GOAL_Y : BLUE_GOAL_Y;

        double deltaX = targetX - robotPose.getX();
        double deltaY = targetY - robotPose.getY();

        return Math.hypot(deltaX, deltaY);
    }

    /**
     * Calculates the target angle for the turret to face the goal.
     * This angle is relative to the robot's current heading.
     * 
     * @param robotPose The current pose of the robot from the follower/pinpoint.
     * @param isRedAlliance True if on Red alliance, False for Blue.
     * @return Target turret angle in radians relative to the robot's front.
     */
    public static double getTargetTurretAngle(Pose robotPose, boolean isRedAlliance) {
        double targetX = isRedAlliance ? RED_GOAL_X : BLUE_GOAL_X;
        double targetY = isRedAlliance ? RED_GOAL_Y : BLUE_GOAL_Y;

        double deltaX = targetX - robotPose.getX();
        double deltaY = targetY - robotPose.getY();

        // Calculate absolute field angle from robot to goal
        double fieldAngleToGoal = Math.atan2(deltaY, deltaX);

        // Subtract robot heading to get robot-relative angle
        double relativeAngle = fieldAngleToGoal - robotPose.getHeading();

        // Removed angle normalization to allow for continuous rotation > 180 degrees
        return relativeAngle;
    }

    /**
     * Helper to keep angles between -180 and 180 degrees (in radians).
     */
    private static double normalizeAngle(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}

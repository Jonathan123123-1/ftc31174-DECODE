package org.firstinspires.ftc.teamcode.shooterConstants;

import com.pedropathing.geometry.Pose;

/**
 * GoalManager
 *
 * Manages the two goal positions (RED and BLUE) for your FTC robot.
 * Automatically selects the correct goal based on alliance color.
 *
 * CONFIGURE THESE COORDINATES FOR YOUR FIELD!
 */
public class GoalManager {

    // ==================== GOAL POSITIONS (CONFIGURE THESE!) ====================

    /**
     * RED GOAL position (x, y in inches, heading doesn't matter)
     * TODO: Measure and set your actual red goal coordinates
     */
    private static final Pose RED_GOAL = new Pose(140.0, 140.0, 0);

    /**
     * BLUE GOAL position (x, y in inches, heading doesn't matter)
     * TODO: Measure and set your actual blue goal coordinates
     */
    private static final Pose BLUE_GOAL = new Pose(13.0, 140.0, 0);

    // ==================== ALLIANCE SELECTION ====================

    public enum Alliance {
        RED,
        BLUE
    }

    private static Alliance currentAlliance = Alliance.BLUE; // Default to blue

    /**
     * Set which alliance you're on.
     * Call this in your OpMode init()!
     *
     * @param alliance RED or BLUE
     */
    public static void setAlliance(Alliance alliance) {
        currentAlliance = alliance;
    }

    /**
     * Get the current goal position based on alliance
     *
     * @return Pose of the current goal
     */
    public static Pose getCurrentGoal() {
        return currentAlliance == Alliance.RED ? RED_GOAL : BLUE_GOAL;
    }

    /**
     * Get the RED goal position
     */
    public static Pose getRedGoal() {
        return RED_GOAL;
    }

    /**
     * Get the BLUE goal position
     */
    public static Pose getBlueGoal() {
        return BLUE_GOAL;
    }

    /**
     * Calculate distance from robot to current goal
     *
     * @param robotPose Current robot position
     * @return Distance in inches
     */
    public static double getDistanceToGoal(Pose robotPose) {
        Pose goal = getCurrentGoal();
        double dx = goal.getX() - robotPose.getX();
        double dy = goal.getY() - robotPose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Calculate angle from robot to current goal
     *
     * @param robotPose Current robot position
     * @return Angle in radians
     */
    public static double getAngleToGoal(Pose robotPose) {
        Pose goal = getCurrentGoal();
        double dx = goal.getX() - robotPose.getX();
        double dy = goal.getY() - robotPose.getY();
        return Math.atan2(dy, dx);
    }

    /**
     * Get current alliance
     */
    public static Alliance getCurrentAlliance() {
        return currentAlliance;
    }
}
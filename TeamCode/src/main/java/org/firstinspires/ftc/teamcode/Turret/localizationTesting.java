package org.firstinspires.ftc.teamcode.Turret;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

public class localizationTesting {

    // --- FIELD CONSTANTS ---
    private static final double GRAVITY = 386.09; // in/s^2
    private static final double LAUNCHER_Z = 14.0; // Height of launcher from floor
    private static final double GOAL_Z = 30.9;

    private static final Pose RED_GOAL = new Pose(-135, -135, GOAL_Z);

    // --- TUNABLE CONSTANTS (From Video) ---
    public static double SCORE_HEIGHT_ADJUST = 5.0; // Height above goal to aim for
    public static double SCORE_ANGLE = Math.toRadians(-12.0); // Entry angle (must be negative)
    public static double PASS_THROUGH_RADIUS = 28.0; // Peak distance from goal

    public static class LaunchVector {
        public final double turretAngle, hoodAngle, flywheelVelocity;
        public LaunchVector(double t, double h, double fv) {
            this.turretAngle = t; this.hoodAngle = h; this.flywheelVelocity = fv;
        }
    }

    public static LaunchVector calculateLaunchVector(Pose robotPose, Vector robotVel, boolean isRed) {
        Pose goal = isRed ? RED_GOAL : new Pose(15, 135, GOAL_Z);

        // 1. Static Math (Assume robot isn't moving first)
        double dx = goal.getX() - robotPose.getX();
        double dy = goal.getY() - robotPose.getY();
        double distToGoal = Math.hypot(dx, dy);
        double thetaToGoal = Math.atan2(dy, dx);

        double x = distToGoal - PASS_THROUGH_RADIUS;
        double y = (GOAL_Z - LAUNCHER_Z) + SCORE_HEIGHT_ADJUST;

        // Calculate theoretical Hood Angle
        double hoodRad = Math.atan((2 * y / x) - Math.tan(SCORE_ANGLE));

        // Calculate theoretical Flywheel Exit Velocity (v0)
        double v0 = Math.sqrt((GRAVITY * x * x) / (2 * Math.pow(Math.cos(hoodRad), 2) * (x * Math.tan(hoodRad) - y)));

        // 2. Velocity Compensation (The Video's "Secret Sauce")
        // Split robot velocity into Parallel (Radial) and Perpendicular (Tangential)
        double relHeading = robotVel.getTheta() - thetaToGoal;
        double vParallel = Math.cos(relHeading) * robotVel.getMagnitude();
        double vPerpendicular = Math.sin(relHeading) * robotVel.getMagnitude();

        // Calculate air time
        double time = x / (v0 * Math.cos(hoodRad));

        // Adjust for robot's radial movement (Parallel)
        double v0_radial = (x / time) - vParallel;

        // Calculate Turret Offset (Perpendicular)
        double turretOffset = Math.atan2(vPerpendicular * time, distToGoal);
        double finalTurret = Math.toDegrees(thetaToGoal - robotPose.getHeading() + turretOffset);

        // Recalculate Final Flywheel and Hood after compensation
        double v0_final = Math.hypot(v0_radial, vPerpendicular / Math.cos(hoodRad));

        return new LaunchVector(normalize(finalTurret), Math.toDegrees(hoodRad), v0_final);
    }

    // --- CONVERSION HELPERS ---
    public static double getFlywheelTicks(double velocity) {
        // Multiplier to convert inches/sec to your motor's Ticks/Sec
        // You mentioned 1570 RPM. If v0 is ~500 in/s, this should be around 70.
        return (72.0 * velocity);
    }

    public static double getHoodServoPos(double degrees) {
        // Map 0-22 degrees to your 0.6-0.8 range
        // If 0 deg = 0.6 and 22 deg = 0.8:
        return 0.6 + (degrees / 22.0) * 0.2;
    }

    private static double normalize(double d) {
        while (d > 180) d -= 360;
        while (d < -180) d += 360;
        return d;
    }
}
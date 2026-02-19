package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.shooterConstants.ShooterConstants;

/**
 * Hybrid Turret Tracking System
 *
 * Uses odometry (Pedro Pathing) as primary source for turret aiming
 * Uses Limelight camera to verify and correct odometry drift
 *
 * WORKFLOW:
 * 1. Robot knows its position from Pedro odometry (x, y, heading)
 * 2. Calculate angle and distance to goal using odometry
 * 3. Use Limelight to verify we're actually aimed correctly
 * 4. If Limelight and odometry disagree too much, trust Limelight to correct
 */
public class HybridTurretTracker {

    // Hardware
    private DcMotorEx turretMotor;
    private DcMotorEx shooterMotor;
    private Servo hoodServo;
    private Limelight3A limelight;
    private Follower follower; // Pedro Pathing odometry

    // Goal position (set based on alliance)
    private double goalX = 15.0;  // Blue goal X (from your constants)
    private double goalY = 135.0; // Blue goal Y

    // Limelight constants (from your TeleOp)
    private static final double CAMERA_HEIGHT_INCHES = 12.0;
    private static final double TARGET_HEIGHT_INCHES = 29.5;
    private static final double CAMERA_MOUNT_ANGLE_DEGREES = 0.0;

    // Turret tracking constants
    private static final double TURRET_TICKS_PER_DEGREE = 1.5; // TUNE THIS - depends on your gearing
    private static final double LIMELIGHT_CORRECTION_THRESHOLD = 5.0; // degrees
    private static final double ODOMETRY_TRUST_DISTANCE = 10.0; // inches - how much we trust odometry

    // PID constants for turret motor
    private static final double TURRET_KP = 0.05;
    private static final double TURRET_HOLD_KP = 0.01;
    private static final double TURRET_HOLD_POWER_MAX = 0.15;

    // Shooter PIDF (from your TeleOp)
    private static final double SHOOTER_KP = 190.0;
    private static final double SHOOTER_KI = 13.0;
    private static final double SHOOTER_KD = 0.0;
    private static final double SHOOTER_KF = 18.0;

    // State tracking
    private boolean isTracking = false;
    private double lastOdometryAngle = 0;
    private double turretAngleOffset = 0; // Limelight correction offset

    public HybridTurretTracker(DcMotorEx turretMotor, DcMotorEx shooterMotor,
                               Servo hoodServo, Limelight3A limelight, Follower follower) {
        this.turretMotor = turretMotor;
        this.shooterMotor = shooterMotor;
        this.hoodServo = hoodServo;
        this.limelight = limelight;
        this.follower = follower;

        // Configure shooter motor with PIDF
        shooterMotor.setVelocityPIDFCoefficients(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF);
    }

    /**
     * Set which goal to track (Blue or Red alliance)
     */
    public void setGoal(boolean isBlueAlliance) {
        if (isBlueAlliance) {
            goalX = 15.0;
            goalY = 135.0;
        } else {
            goalX = 135.0;
            goalY = 135.0;
        }
    }

    /**
     * Main update loop - call this every loop iteration
     * Returns true if turret is aimed and ready to shoot
     */
    public boolean update(OpMode opMode) {
        if (!isTracking) {
            turretMotor.setPower(0);
            return false;
        }

        // Get current robot pose from Pedro odometry
        Pose currentPose = follower.getPose();
        double robotX = currentPose.getX();
        double robotY = currentPose.getY();
        double robotHeading = Math.toDegrees(currentPose.getHeading());

        // Calculate distance and angle to goal using odometry
        double dx = goalX - robotX;
        double dy = goalY - robotY;
        double distanceToGoal = Math.sqrt(dx * dx + dy * dy);
        double angleToGoal = Math.toDegrees(Math.atan2(dy, dx));

        // Calculate required turret angle (relative to robot heading)
        double requiredTurretAngle = normalizeAngle(angleToGoal - robotHeading);

        // Get Limelight correction if available
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double limelightTx = result.getTx(); // Horizontal offset in degrees
            double limelightDistance = calculateLimelightDistance(result.getTy());

            // Check if odometry and Limelight agree
            double discrepancy = Math.abs(limelightTx);

            if (discrepancy > LIMELIGHT_CORRECTION_THRESHOLD) {
                // Odometry is off - apply Limelight correction
                turretAngleOffset += limelightTx * 0.1; // Gentle correction
                opMode.telemetry.addData("Tracking Mode", "LIMELIGHT CORRECTING");
            } else {
                // Odometry and Limelight agree - trust odometry
                turretAngleOffset *= 0.95; // Slowly decay offset
                opMode.telemetry.addData("Tracking Mode", "ODOMETRY PRIMARY");
            }

            // Compare distances
            double distanceDiscrepancy = Math.abs(distanceToGoal - limelightDistance);
            if (distanceDiscrepancy > ODOMETRY_TRUST_DISTANCE) {
                // Use Limelight distance if odometry seems way off
                distanceToGoal = limelightDistance;
                opMode.telemetry.addData("Distance Source", "Limelight");
            } else {
                opMode.telemetry.addData("Distance Source", "Odometry");
            }

            opMode.telemetry.addData("Limelight Tx", "%.2f°", limelightTx);
            opMode.telemetry.addData("Limelight Dist", "%.1f in", limelightDistance);
        } else {
            // No Limelight - pure odometry mode
            opMode.telemetry.addData("Tracking Mode", "ODOMETRY ONLY");
        }

        // Apply correction offset
        double finalTurretAngle = requiredTurretAngle + turretAngleOffset;

        // Control turret motor to aim
        double currentTurretTicks = turretMotor.getCurrentPosition();
        double targetTicks = finalTurretAngle * TURRET_TICKS_PER_DEGREE;
        double error = targetTicks - currentTurretTicks;

        // Proportional control
        double power = error * TURRET_KP;
        power = Math.max(-0.7, Math.min(0.7, power)); // Limit power

        // Apply holding power when close
        if (Math.abs(error) < 10) {
            double holdPower = error * TURRET_HOLD_KP;
            power = Math.max(-TURRET_HOLD_POWER_MAX, Math.min(TURRET_HOLD_POWER_MAX, holdPower));
        }

        turretMotor.setPower(power);

        // Set shooter RPM and hood based on distance
        double targetRPM = ShooterConstants.targetRPM(distanceToGoal);
        double hoodPosition = ShooterConstants.hoodPosition(distanceToGoal);

        shooterMotor.setVelocity(targetRPM);
        hoodServo.setPosition(hoodPosition);

        // Telemetry
        opMode.telemetry.addData("Robot Pos", "(%.1f, %.1f) @ %.1f°", robotX, robotY, robotHeading);
        opMode.telemetry.addData("Goal Pos", "(%.1f, %.1f)", goalX, goalY);
        opMode.telemetry.addData("Distance", "%.1f in", distanceToGoal);
        opMode.telemetry.addData("Turret Angle", "%.1f° (target: %.1f°)",
                currentTurretTicks / TURRET_TICKS_PER_DEGREE, finalTurretAngle);
        opMode.telemetry.addData("Correction Offset", "%.2f°", turretAngleOffset);
        opMode.telemetry.addData("Target RPM", "%.0f", targetRPM);
        opMode.telemetry.addData("Hood Position", "%.3f", hoodPosition);
        opMode.telemetry.addData("Turret Error", "%.1f ticks", error);

        // Check if ready to shoot
        boolean turretReady = Math.abs(error) < 10; // Within 10 ticks
        boolean shooterReady = Math.abs(shooterMotor.getVelocity() - targetRPM) < 50; // Within 50 RPM

        opMode.telemetry.addData("Ready to Shoot", turretReady && shooterReady);

        return turretReady && shooterReady;
    }

    /**
     * Calculate distance using Limelight vertical angle
     */
    private double calculateLimelightDistance(double ty) {
        return (TARGET_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) /
                Math.tan(Math.toRadians(CAMERA_MOUNT_ANGLE_DEGREES + ty));
    }

    /**
     * Normalize angle to [-180, 180] range
     */
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    /**
     * Enable/disable tracking
     */
    public void setTracking(boolean tracking) {
        this.isTracking = tracking;
        if (!tracking) {
            turretMotor.setPower(0);
            shooterMotor.setVelocity(0);
        }
    }

    /**
     * Reset turret encoder and correction offset
     */
    public void reset() {
        turretAngleOffset = 0;
    }

    /**
     * Get current distance to goal (useful for autonomous)
     */
    public double getDistanceToGoal() {
        Pose currentPose = follower.getPose();
        double dx = goalX - currentPose.getX();
        double dy = goalY - currentPose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Get current angle to goal (useful for autonomous)
     */
    public double getAngleToGoal() {
        Pose currentPose = follower.getPose();
        double dx = goalX - currentPose.getX();
        double dy = goalY - currentPose.getY();
        return Math.toDegrees(Math.atan2(dy, dx));
    }
}
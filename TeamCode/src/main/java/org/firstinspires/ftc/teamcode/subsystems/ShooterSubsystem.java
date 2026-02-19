package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.subsystems.ShooterConstants.*;

/**
 * Shooter subsystem with physics-based trajectory calculation
 * Manages turret rotation, hood angle, and flywheel speed
 * Integrated with Pedro Pathing
 */
public class ShooterSubsystem {

    // Hardware
    private final DcMotorEx turretMotor;
    private final DcMotorEx flywheelMotor;
    private final Servo hoodServo;

    // Simple PID implementation
    private final SimplePID turretPID;
    private final SimplePID flywheelPID;

    // State
    private ShooterPhysics.ShooterSolution currentSolution;
    private boolean isReadyToShoot = false;

    // Target values
    private double targetTurretAngle = 0.0; // degrees, robot-relative
    private double targetFlywheelTicksPerSec = 0.0;
    private double targetHoodPosition = MIN_SERVO_POSITION;

    /**
     * Simple PID Controller implementation
     */
    private static class SimplePID {
        private final double kP, kI, kD;
        private double lastError = 0;
        private double integral = 0;

        public SimplePID(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }

        public double calculate(double current, double target) {
            double error = target - current;
            integral += error;
            double derivative = error - lastError;
            lastError = error;

            return kP * error + kI * integral + kD * derivative;
        }

        public void reset() {
            lastError = 0;
            integral = 0;
        }
    }

    /**
     * Initialize the shooter subsystem
     */
    public ShooterSubsystem(HardwareMap hardwareMap) {
        // Initialize hardware
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        hoodServo = hardwareMap.get(Servo.class, "turret_servo");

        // Configure motors
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize PID controllers
        turretPID = new SimplePID(TURRET_KP, TURRET_KI, TURRET_KD);
        flywheelPID = new SimplePID(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);

        // Initialize state
        currentSolution = new ShooterPhysics.ShooterSolution();

        // Set hood to home position
        hoodServo.setPosition(MIN_SERVO_POSITION);
    }

    /**
     * Calculate and set shooter parameters for a shot
     *
     * @param robotX current robot X position (inches, 0-144)
     * @param robotY current robot Y position (inches, 0-144)
     * @param robotHeading current robot heading (radians)
     * @param robotVx robot velocity in X (in/s)
     * @param robotVy robot velocity in Y (in/s)
     * @param isRedAlliance true for red alliance, false for blue
     */
    public void calculateShot(double robotX, double robotY, double robotHeading,
                              double robotVx, double robotVy, boolean isRedAlliance) {

        // Calculate shooter solution
        currentSolution = ShooterPhysics.calculateShooterSolution(
                robotX, robotY, robotHeading, robotVx, robotVy, isRedAlliance
        );

        if (currentSolution.isValid) {
            // Set targets
            targetTurretAngle = currentSolution.turretAngle; // Already in degrees
            targetFlywheelTicksPerSec = currentSolution.flywheelTicksPerSec;
            targetHoodPosition = currentSolution.hoodServoPosition;
        }
    }

    /**
     * Update shooter motors and servos
     * Call this in your periodic/loop method
     */
    public void update() {
        // Update turret position
        updateTurret();

        // Update flywheel speed
        updateFlywheel();

        // Update hood position
        hoodServo.setPosition(targetHoodPosition);

        // Check if shooter is ready
        updateReadyState();
    }

    /**
     * Update turret motor with PID control
     */
    private void updateTurret() {
        // Get current position in degrees
        double currentTicks = turretMotor.getCurrentPosition();
        double currentAngleDeg = ticksToDegrees(currentTicks);

        // Calculate PID output
        double pidOutput = turretPID.calculate(currentAngleDeg, targetTurretAngle);

        // Clamp and apply power
        pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));
        turretMotor.setPower(pidOutput);
    }

    /**
     * Update flywheel motor with PID + feedforward control
     */
    private void updateFlywheel() {
        // Get current velocity in ticks/sec
        double currentTicksPerSec = flywheelMotor.getVelocity();

        // Calculate PID output
        double pidOutput = flywheelPID.calculate(currentTicksPerSec, targetFlywheelTicksPerSec);

        // Add feedforward term
        double feedforward = FLYWHEEL_KF * (targetFlywheelTicksPerSec / (MAX_FLYWHEEL_RPM * FLYWHEEL_TICKS_PER_REV / 60.0));

        // Combine and apply
        double power = pidOutput + feedforward;
        power = Math.max(0.0, Math.min(1.0, power)); // Flywheel only spins one direction

        flywheelMotor.setPower(power);
    }

    /**
     * Check if shooter is ready to fire
     */
    private void updateReadyState() {
        // Check turret position
        double currentTurretAngle = ticksToDegrees(turretMotor.getCurrentPosition());
        boolean turretReady = Math.abs(currentTurretAngle - targetTurretAngle) < Math.toDegrees(TURRET_ANGLE_TOLERANCE);

        // Check flywheel speed
        double currentTicksPerSec = flywheelMotor.getVelocity();
        double currentRPM = ShooterPhysics.ticksPerSecToRPM(currentTicksPerSec);
        double targetRPM = ShooterPhysics.ticksPerSecToRPM(targetFlywheelTicksPerSec);
        boolean flywheelReady = Math.abs(currentRPM - targetRPM) < FLYWHEEL_RPM_TOLERANCE;

        // Check hood position
        boolean hoodReady = Math.abs(hoodServo.getPosition() - targetHoodPosition) < HOOD_POSITION_TOLERANCE;

        // Check if solution is valid
        isReadyToShoot = currentSolution.isValid && turretReady && flywheelReady && hoodReady;
    }

    /**
     * Convert motor ticks to degrees
     */
    private double ticksToDegrees(double ticks) {
        double revolutions = ticks / TURRET_TICKS_PER_REV / TURRET_GEAR_RATIO;
        return revolutions * 360.0;
    }

    /**
     * Check if shooter is ready to fire
     */
    public boolean isReady() {
        return isReadyToShoot;
    }

    /**
     * Get current shooter solution for telemetry
     */
    public ShooterPhysics.ShooterSolution getCurrentSolution() {
        return currentSolution;
    }

    /**
     * Stop all motors
     */
    public void stop() {
        turretMotor.setPower(0);
        flywheelMotor.setPower(0);
        targetFlywheelTicksPerSec = 0;
        targetTurretAngle = 0;
    }

    /**
     * Manual control - set hood position directly
     */
    public void setHoodPosition(double position) {
        targetHoodPosition = Math.max(MIN_SERVO_POSITION, Math.min(MAX_SERVO_POSITION, position));
        hoodServo.setPosition(targetHoodPosition);
    }

    /**
     * Manual control - set flywheel power directly
     */
    public void setFlywheelPower(double power) {
        flywheelMotor.setPower(Math.max(0.0, Math.min(1.0, power)));
    }

    /**
     * Manual control - set turret power directly
     */
    public void setTurretPower(double power) {
        turretMotor.setPower(Math.max(-1.0, Math.min(1.0, power)));
    }

    /**
     * Reset turret encoder to zero at current position
     */
    public void resetTurretEncoder() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretPID.reset();
    }

    /**
     * Get telemetry data
     */
    public String getTelemetry() {
        double currentTurretAngle = ticksToDegrees(turretMotor.getCurrentPosition());
        double currentTicksPerSec = flywheelMotor.getVelocity();
        double currentRPM = ShooterPhysics.ticksPerSecToRPM(currentTicksPerSec);
        double targetRPM = ShooterPhysics.ticksPerSecToRPM(targetFlywheelTicksPerSec);

        return String.format(Locale.US,
                "Shooter Status:\n" +
                        "Ready: %s\n" +
                        "Turret: %.1f° (target: %.1f°)\n" +
                        "Flywheel: %.0f RPM (target: %.0f RPM)\n" +
                        "Hood: %.3f (target: %.3f)\n" +
                        "Launch Angle: %.1f°\n" +
                        "Launch Velocity: %.1f in/s",
                isReadyToShoot,
                currentTurretAngle,
                targetTurretAngle,
                currentRPM,
                targetRPM,
                hoodServo.getPosition(),
                targetHoodPosition,
                Math.toDegrees(currentSolution.launchAngle),
                currentSolution.launchVelocity
        );
    }
}
package org.firstinspires.ftc.teamcode.testing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooterConstants.AdvancedShooterSystem;

/**
 * COMPLETE ADVANCED SHOOTING TEST
 *
 * Features:
 * - Your 108-point lookup table
 * - Cube Robot velocity compensation
 * - Air resistance simulation
 * - Turret angle constraints
 * - Red/Blue goal switching
 * - Automatic aiming
 *
 * TOTALLY SEPARATE FROM YOUR EXISTING CODE!
 *
 * CONTROLS:
 * Gamepad 1:
 *   Left Stick: Drive/strafe
 *   Right Stick X: Rotate
 *   A: Slow mode
 *   B: Fast mode
 *   X: Toggle field-oriented
 *   Right Bumper: Intake reverse
 *   Right Trigger: Intake forward
 *
 * Gamepad 2:
 *   A: Set BLUE alliance
 *   B: Set RED alliance
 *   Left Bumper: Start auto-aiming
 *   Right Bumper: FIRE (when ready)
 *   Back: Stop aiming
 *   Right Trigger: Manual intake
 */
@TeleOp(name = "🎯 ADVANCED SHOOTER TEST", group = "TESTING")
public class AdvancedShooterTest extends LinearOpMode {

    // ==================== HARDWARE ====================
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private IMU imu;
    private DcMotorEx shooterMotor;
    private DcMotorEx intakeMotor;
    private DcMotorEx turretMotor;
    private Servo hoodServo;
    private Servo turretStopperServo;
    private Follower follower;

    // ==================== STATE ====================
    private boolean isAiming = false;
    private boolean isReadyToShoot = false;
    private boolean isShooting = false;
    private AdvancedShooterSystem.ShooterParams currentShot = null;

    private ElapsedTime aimTimer = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();

    private double maxSpeed = 0.77;
    private boolean fieldOriented = false;
    private boolean lastFieldToggle = false;

    // Turret state
    private double currentTurretAngle = 0; // Field-relative
    private int turretTargetTicks = 0;

    // ==================== CONSTANTS ====================
    private static final double SHOOTER_kP = 180.0;
    private static final double SHOOTER_kI = 13.0;
    private static final double SHOOTER_kD = 0.0;
    private static final double SHOOTER_kF = 18.0;

    private static final double INTAKE_VELOCITY = 1800.0;
    private static final double TURRET_HOME = 0.45;
    private static final double TURRET_ACTIVE = 0.2;

    private static final double RPM_TOLERANCE = 50.0;
    private static final double MIN_AIM_TIME = 0.3;

    // TURRET MOTOR CONSTANTS - TUNE THESE!
    private static final double TURRET_TICKS_PER_RADIAN = 100.0; // TODO: Calibrate this!
    private static final double TURRET_POWER = 0.6;
    private static final int TURRET_TOLERANCE_TICKS = 10;

    @Override
    public void runOpMode() {
        initializeHardware();

        // Set default alliance
        AdvancedShooterSystem.setAlliance(AdvancedShooterSystem.Alliance.BLUE);

        telemetry.addLine("════════════════════════════════");
        telemetry.addLine("   ADVANCED SHOOTER TEST");
        telemetry.addLine("════════════════════════════════");
        telemetry.addLine();
        telemetry.addLine("✓ Pinpoint localization");
        telemetry.addLine("✓ 108-point lookup table");
        telemetry.addLine("✓ Velocity compensation");
        telemetry.addLine("✓ Air resistance simulation");
        telemetry.addLine("✓ Turret constraints");
        telemetry.addLine();
        telemetry.addLine("Press START");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update odometry (Pinpoint through Pedro Pathing)
            follower.update();
            Pose robotPose = follower.getPose();
            Vector velocity = follower.getVelocity();
            double velocityX = velocity.getXComponent();
            double velocityY = velocity.getYComponent();

            double velocityMag = Math.sqrt(
                    velocityX * velocityX +
                            velocityY * velocityY
            );

            // ===== ALLIANCE SELECTION =====
            if (gamepad2.a) {
                AdvancedShooterSystem.setAlliance(AdvancedShooterSystem.Alliance.BLUE);
                telemetry.speak("Blue alliance");
            }
            if (gamepad2.b) {
                AdvancedShooterSystem.setAlliance(AdvancedShooterSystem.Alliance.RED);
                telemetry.speak("Red alliance");
            }

            // ===== AIMING =====
            if (gamepad2.left_bumper && !isAiming && !isShooting) {
                startAiming();
            }

            if (isAiming && !isShooting) {
                updateAiming(robotPose, velocityX, velocityY);
            }

            if (gamepad2.back && isAiming) {
                stopAiming();
            }

            // ===== SHOOTING =====
            if (gamepad2.right_bumper && isReadyToShoot && !isShooting) {
                startShooting();
            }

            if (isShooting) {
                updateShooting();
            }

            // ===== TURRET CONTROL =====
            if (isAiming || isReadyToShoot) {
                updateTurretMotor(robotPose.getHeading());
            }

            // ===== INTAKE =====
            if (!isShooting) {
                if (gamepad1.right_bumper) {
                    intakeMotor.setVelocity(-INTAKE_VELOCITY);
                } else if (gamepad1.right_trigger > 0.2) {
                    intakeMotor.setVelocity(INTAKE_VELOCITY);
                } else if (gamepad2.right_trigger > 0.8) {
                    intakeMotor.setVelocity(INTAKE_VELOCITY);
                } else {
                    intakeMotor.setVelocity(0);
                }
            }

            // ===== DRIVE =====
            handleDrive();

            // ===== TELEMETRY =====
            displayTelemetry(robotPose, velocityMag);
        }
    }

    // ==================== SHOOTING LOGIC ====================

    private void startAiming() {
        isAiming = true;
        isReadyToShoot = false;
        aimTimer.reset();
        telemetry.speak("Aiming");
    }

    private void updateAiming(Pose robotPose, double vx, double vy) {
        // Calculate shot parameters using the advanced system
        currentShot = AdvancedShooterSystem.calculate(robotPose, vx, vy);

        if (!currentShot.valid) {
            stopAiming();
            telemetry.speak("Invalid shot");
            return;
        }

        // Check if turret angle is safe (won't hit gear/wiring)
        if (!currentShot.isTurretAngleSafe()) {
            telemetry.speak("Turret angle unsafe");
            // Could stop here or try to continue with clamped angle
        }

        // Apply shooter settings
        shooterMotor.setVelocity(currentShot.rpm);
        hoodServo.setPosition(currentShot.hoodPosition);

        // Set turret target
        currentTurretAngle = currentShot.turretAngle;

        // Convert field-relative angle to robot-relative for motor
        double robotRelativeAngle = AdvancedShooterSystem.fieldToRobotRelative(
                currentShot.turretAngle,
                robotPose.getHeading()
        );

        turretTargetTicks = (int)(robotRelativeAngle * TURRET_TICKS_PER_RADIAN);

        // Check if ready
        double currentRPM = shooterMotor.getVelocity();
        boolean rpmReady = Math.abs(currentRPM - currentShot.rpm) < RPM_TOLERANCE;
        boolean timeReady = aimTimer.seconds() > MIN_AIM_TIME;
        boolean turretReady = isTurretAtTarget();

        if (rpmReady && timeReady && turretReady && !isReadyToShoot) {
            isReadyToShoot = true;
            telemetry.speak("Ready");
        }
    }

    private void stopAiming() {
        isAiming = false;
        isReadyToShoot = false;
        currentShot = null;
        shooterMotor.setVelocity(0);
        turretMotor.setPower(0);
    }

    private void startShooting() {
        isShooting = true;
        shootTimer.reset();
        telemetry.speak("Shooting");
    }

    private void updateShooting() {
        // Open stopper after delay
        if (shootTimer.seconds() > 0.3) {
            turretStopperServo.setPosition(TURRET_ACTIVE);
            intakeMotor.setVelocity(-INTAKE_VELOCITY);
        }

        // End shooting sequence
        if (shootTimer.seconds() > 2.0) {
            endShooting();
        }
    }

    private void endShooting() {
        isShooting = false;
        turretStopperServo.setPosition(TURRET_HOME);
        intakeMotor.setVelocity(0);

        // Resume aiming if still active
        if (isAiming) {
            isReadyToShoot = false;
            aimTimer.reset();
        } else {
            stopAiming();
        }
    }

    // ==================== TURRET MOTOR CONTROL ====================

    private void updateTurretMotor(double robotHeading) {
        int currentTicks = turretMotor.getCurrentPosition();
        int error = turretTargetTicks - currentTicks;

        if (Math.abs(error) < TURRET_TOLERANCE_TICKS) {
            // At target - hold position
            turretMotor.setPower(0);
            return;
        }

        // Move toward target
        double power = TURRET_POWER;

        // Slow down as we approach
        if (Math.abs(error) < 100) {
            power = 0.3 + (Math.abs(error) / 100.0) * (TURRET_POWER - 0.3);
        }

        turretMotor.setPower(error > 0 ? power : -power);
    }

    private boolean isTurretAtTarget() {
        int currentTicks = turretMotor.getCurrentPosition();
        int error = Math.abs(turretTargetTicks - currentTicks);
        return error < TURRET_TOLERANCE_TICKS;
    }

    // ==================== HARDWARE & DRIVE ====================

    private void initializeHardware() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        hoodServo = hardwareMap.get(Servo.class, "turret_servo");
        turretStopperServo = hardwareMap.get(Servo.class, "turretStopper");

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setVelocityPIDFCoefficients(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretStopperServo.setPosition(TURRET_HOME);

        follower = Constants.createFollower(hardwareMap, telemetry);
        follower.setStartingPose(new Pose(72, 72, 0));
    }

    private void handleDrive() {
        boolean fieldToggle = gamepad1.x;
        if (fieldToggle && !lastFieldToggle) {
            fieldOriented = !fieldOriented;
        }
        lastFieldToggle = fieldToggle;

        if (gamepad1.a) maxSpeed = 0.6;
        if (gamepad1.b) maxSpeed = 0.76;

        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        if (fieldOriented) {
            double heading = imu.getRobotYawPitchRollAngles().getYaw();
            double temp = forward * Math.cos(heading) + right * Math.sin(heading);
            right = -forward * Math.sin(heading) + right * Math.cos(heading);
            forward = temp;
        }

        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double bl = forward - right + rotate;
        double br = forward + right - rotate;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max; fr /= max; bl /= max; br /= max;
        }

        frontLeftDrive.setPower(fl * maxSpeed);
        frontRightDrive.setPower(fr * maxSpeed);
        backLeftDrive.setPower(bl * maxSpeed);
        backRightDrive.setPower(br * maxSpeed);
    }

    // ==================== TELEMETRY ====================

    private void displayTelemetry(Pose robotPose, double velocityMag) {
        telemetry.addLine("════════════════════════════════");
        telemetry.addLine("   ADVANCED SHOOTER TEST");
        telemetry.addLine("════════════════════════════════");

        telemetry.addData("Alliance", AdvancedShooterSystem.getAlliance());
        telemetry.addLine();

        telemetry.addLine("─── STATUS ───");
        telemetry.addData("Aiming", isAiming ? "YES" : "NO");
        telemetry.addData("Ready", isReadyToShoot ? "✓ READY TO FIRE" : "X Not Ready");
        telemetry.addData("Shooting", isShooting ? "FIRING!" : "No");
        telemetry.addLine();

        if (currentShot != null && isAiming) {
            telemetry.addLine("─── SHOT PARAMETERS ───");
            telemetry.addData("Method", currentShot.method);
            telemetry.addData("Velocity Comp", currentShot.velocityCompensated ? "YES" : "NO");
            telemetry.addData("Target RPM", "%.0f", currentShot.rpm);
            telemetry.addData("Actual RPM", "%.0f", shooterMotor.getVelocity());
            telemetry.addData("Hood", "%.3f", currentShot.hoodPosition);
            telemetry.addData("Distance", "%.1f\"", currentShot.distance);
            telemetry.addData("Time of Flight", "%.3fs", currentShot.timeOfFlight);
            telemetry.addData("Turret Angle", "%.1f°", Math.toDegrees(currentShot.turretAngle));
            telemetry.addData("Turret Safe", currentShot.isTurretAngleSafe() ? "YES" : "⚠ NO");
            telemetry.addLine();
        }

        telemetry.addLine("─── ROBOT STATE ───");
        telemetry.addData("Position", "X:%.1f Y:%.1f", robotPose.getX(), robotPose.getY());
        telemetry.addData("Velocity", "%.1f in/s", velocityMag);
        telemetry.addData("Turret Ticks", "%d / %d",
                turretMotor.getCurrentPosition(), turretTargetTicks);
        telemetry.addLine();

        telemetry.addLine("─── CONTROLS ───");
        telemetry.addLine("L Bumper: Aim | R Bumper: Fire");
        telemetry.addLine("A: Blue | B: Red");
        telemetry.addLine("Back: Stop Aiming");

        telemetry.update();
    }
}

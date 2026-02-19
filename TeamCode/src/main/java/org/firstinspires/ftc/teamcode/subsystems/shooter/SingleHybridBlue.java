package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.shooterConstants.ShooterConstants;

/**
 * HYBRID TRACKING SINGLE CONTROLLER - BLUE ALLIANCE
 *
 * ALL CONTROLS ON GAMEPAD 1:
 * - Left Stick: Drive (forward/strafe)
 * - Right Stick X: Rotate
 * - A: Slow speed (0.6)
 * - B: Fast speed (0.76)
 * - X: Toggle field-oriented
 * - Y: Toggle shooter on/off
 *
 * - Left Bumper: AUTO-AIM (holds tracking active)
 * - Right Bumper: SHOOT (opens stopper)
 * - Left Trigger: Intake IN
 * - Right Trigger: Intake OUT
 *
 * - D-pad UP: Switch to FAR shot
 * - D-pad DOWN: Switch to CLOSE shot
 * - D-pad LEFT: Toggle LEGACY mode (camera only)
 * - D-pad RIGHT: Toggle HYBRID mode (odometry + camera)
 */
@TeleOp(name = "SINGLE CONTROLLER Hybrid Blue", group = "TEST")
public class SingleHybridBlue extends LinearOpMode {

    /* ================= DRIVE ================= */
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private IMU imu;
    private Follower follower;

    private double maxSpeed = 0.77;
    private boolean fieldOriented = false;
    private boolean lastFieldToggle = false;

    /* ================= MECHANISMS ================= */
    private DcMotorEx shooterMotor;
    private DcMotorEx intakeMotor;
    private Servo turretServo;
    private Servo hoodServo;
    private DcMotorEx turretMotor;
    private Limelight3A limelight;

    private boolean shooterOn = false;
    private boolean lastShooterToggle = false;

    /* ================= TURRET STATE ================= */
    private boolean turretActive = false;
    private long turretStartTime = 0;
    private boolean hybridTrackingActive = true; // Start with HYBRID mode

    /* ================= SHOOTER CONSTANTS ================= */
    private static final double FAR_VELOCITY = 1515;
    private static final double CLOSE_VELOCITY = 1280;

    private static final double SHOOTER_kP = 180.0;
    private static final double SHOOTER_kI = 13.0;
    private static final double SHOOTER_kD = 0.0;
    private static final double SHOOTER_kF = 18;

    private double shooterTargetVelocity = CLOSE_VELOCITY;

    /* ================= OTHER CONSTANTS ================= */
    private static final double INTAKE_VELOCITY = 1800;
    private static final double TURRET_HOME = 0.45;
    private static final double TURRET_ACTIVE = 0.2;
    private static final long TURRET_TIME_MS = 1500;

    /* ================= LIMELIGHT CONSTANTS ================= */
    private static final double CAMERA_HEIGHT_INCHES = 12.0;
    private static final double TARGET_HEIGHT_INCHES = 29.5;
    private static final double CAMERA_MOUNT_ANGLE_DEGREES = 0.0;
    private static final double LimeLight_kP = 0.008;

    /* ================= HYBRID TRACKING CONSTANTS - TUNE THESE! ================= */
    private static final double BLUE_GOAL_X = 15.0;
    private static final double BLUE_GOAL_Y = 135.0;

    // CRITICAL: Tune this value! See calibration instructions below
    private static final double TURRET_TICKS_PER_DEGREE = 1.5;

    private static final double TURRET_HYBRID_KP = 0.05;
    private static final double LIMELIGHT_CORRECTION_THRESHOLD = 5.0;
    private static final double CORRECTION_RATE = 0.15; // How fast to apply Limelight corrections

    private double turretAngleOffset = 0;

    @Override
    public void runOpMode() {
        /* ---------- Hardware Init ---------- */
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(8); // Blue pipeline
        limelight.start();

        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        // Initialize Pedro Pathing
        follower = Constants.createFollower(hardwareMap);
        // SET YOUR STARTING POSITION HERE
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(180)));

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        turretServo = hardwareMap.get(Servo.class, "turretStopper");
        hoodServo = hardwareMap.get(Servo.class, "turret_servo");

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setVelocityPIDFCoefficients(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretServo.setDirection(Servo.Direction.REVERSE);
        turretServo.setPosition(TURRET_HOME);

        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("SINGLE CONTROLLER HYBRID (BLUE)");
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("LEFT BUMPER: Auto-aim");
        telemetry.addLine("RIGHT BUMPER: Shoot");
        telemetry.addLine("D-PAD LEFT: Legacy mode");
        telemetry.addLine("D-PAD RIGHT: Hybrid mode");
        telemetry.addLine("═══════════════════════════════");
        telemetry.update();

        waitForStart();

        /* ================= MAIN LOOP ================= */
        while (opModeIsActive()) {
            follower.update(); // Update odometry

            LLResult result = limelight.getLatestResult();
            double distance = 180;
            if (result.isValid()) {
                double ty = result.getTy();
                distance = (TARGET_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) /
                        Math.tan(Math.toRadians(CAMERA_MOUNT_ANGLE_DEGREES + ty));
            }

            /* ---------- MODE SWITCHING ---------- */
            if (gamepad1.dpad_left) {
                hybridTrackingActive = false;
                turretAngleOffset = 0;
                sleep(200);
            }
            if (gamepad1.dpad_right) {
                hybridTrackingActive = true;
                turretAngleOffset = 0;
                sleep(200);
            }

            /* ---------- DRIVE ---------- */
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

            drive(forward, right, rotate);

            /* ---------- SHOOTER ZONE SELECT ---------- */
            if (gamepad1.dpad_up) shooterTargetVelocity = FAR_VELOCITY;
            if (gamepad1.dpad_down) shooterTargetVelocity = CLOSE_VELOCITY;

            /* ---------- SHOOTER TOGGLE ---------- */
            boolean shooterToggle = gamepad1.y;
            if (shooterToggle && !lastShooterToggle) {
                shooterOn = !shooterOn;
            }
            lastShooterToggle = shooterToggle;

            if (shooterOn) {
                shooterMotor.setVelocity(shooterTargetVelocity);
            } else {
                shooterMotor.setVelocity(0);
            }

            /* ---------- INTAKE ---------- */
            if (gamepad1.left_trigger > 0.2) {
                intakeMotor.setVelocity(INTAKE_VELOCITY); // Intake IN
            } else if (gamepad1.right_trigger > 0.2) {
                intakeMotor.setVelocity(-INTAKE_VELOCITY); // Intake OUT
            } else {
                intakeMotor.setVelocity(0);
            }

            /* ---------- TURRET SERVO (SHOOT) ---------- */
            if (gamepad1.right_bumper && !turretActive) {
                turretServo.setPosition(TURRET_ACTIVE);
                turretStartTime = System.currentTimeMillis();
                turretActive = true;
            }

            if (turretActive && System.currentTimeMillis() - turretStartTime >= TURRET_TIME_MS) {
                turretServo.setPosition(TURRET_HOME);
                turretActive = false;
            }

            /* ---------- TURRET TRACKING (AUTO-AIM) ---------- */
            if (gamepad1.left_bumper) {
                // Set shooter settings based on distance
                hoodServo.setPosition(ShooterConstants.hoodPosition(distance));
                shooterTargetVelocity = ShooterConstants.targetRPM(distance);

                if (hybridTrackingActive) {
                    updateHybridTracking(result, distance);
                } else {
                    updateLegacyTracking(result);
                }
            } else {
                turretMotor.setPower(0);
            }

            /* ---------- TELEMETRY ---------- */
            telemetry.addData("MODE", hybridTrackingActive ? "🟢 HYBRID" : "🔵 LEGACY");
            telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
            telemetry.addData("Speed", maxSpeed == 0.6 ? "SLOW" : "FAST");
            telemetry.addLine("---");

            if (hybridTrackingActive) {
                Pose currentPose = follower.getPose();
                telemetry.addData("Robot", "(%.0f, %.0f) @ %.0f°",
                        currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
                telemetry.addData("Odo Dist", "%.0f in", getOdometryDistanceToGoal());
                telemetry.addData("Correction", "%.1f°", turretAngleOffset);
            }

            telemetry.addData("LL Dist", result.isValid() ? String.format("%.0f in", distance) : "NO TARGET");
            telemetry.addData("Target RPM", "%.0f", shooterTargetVelocity);
            telemetry.addData("Actual RPM", "%.0f", shooterMotor.getVelocity());
            telemetry.addData("Turret Ticks", turretMotor.getCurrentPosition());

            telemetry.update();
        }
    }

    /**
     * HYBRID MODE: Odometry + Limelight correction
     */
    private void updateHybridTracking(LLResult result, double distance) {
        Pose currentPose = follower.getPose();
        double robotX = currentPose.getX();
        double robotY = currentPose.getY();
        double robotHeading = Math.toDegrees(currentPose.getHeading());

        // Calculate angle to goal from odometry
        double dx = BLUE_GOAL_X - robotX;
        double dy = BLUE_GOAL_Y - robotY;
        double angleToGoal = Math.toDegrees(Math.atan2(dy, dx));
        double requiredTurretAngle = normalizeAngle(angleToGoal - robotHeading);

        // Apply Limelight correction
        if (result != null && result.isValid()) {
            double limelightTx = result.getTx();

            if (Math.abs(limelightTx) > LIMELIGHT_CORRECTION_THRESHOLD) {
                // Odometry is drifting - correct it
                turretAngleOffset += limelightTx * CORRECTION_RATE;
            } else {
                // On target - slowly decay offset
                turretAngleOffset *= 0.92;
            }
        }

        // Final angle with correction
        double finalTurretAngle = requiredTurretAngle + turretAngleOffset;

        // Control turret motor
        double currentTicks = turretMotor.getCurrentPosition();
        double targetTicks = finalTurretAngle * TURRET_TICKS_PER_DEGREE;
        double error = targetTicks - currentTicks;

        double power = error * TURRET_HYBRID_KP;
        power = Math.max(-0.7, Math.min(0.7, power));

        turretMotor.setPower(power);
    }

    /**
     * LEGACY MODE: Pure Limelight tracking
     */
    private void updateLegacyTracking(LLResult result) {
        if (result.isValid() && Math.abs(result.getTx()) > 3) {
            turretMotor.setPower(-result.getTx() * LimeLight_kP + Math.signum(-result.getTx()) * 0.085);
        } else {
            turretMotor.setPower(0);
        }
    }

    private double getOdometryDistanceToGoal() {
        Pose currentPose = follower.getPose();
        double dx = BLUE_GOAL_X - currentPose.getX();
        double dy = BLUE_GOAL_Y - currentPose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private void drive(double forward, double right, double rotate) {
        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double bl = forward - right + rotate;
        double br = forward + right - rotate;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        frontLeftDrive.setPower(fl * maxSpeed);
        frontRightDrive.setPower(fr * maxSpeed);
        backLeftDrive.setPower(bl * maxSpeed);
        backRightDrive.setPower(br * maxSpeed);
    }
}
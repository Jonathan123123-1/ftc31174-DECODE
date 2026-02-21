package org.firstinspires.ftc.teamcode.testing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.pedroPathing.teleConstants;
import org.firstinspires.ftc.teamcode.shooterConstants.TurretAiming;

/**
 * STEP 1: Test Turret Aiming ONLY
 *
 * This OpMode ONLY tests turret aiming.
 * No shooting, no intake - just turret rotation.
 *
 * CONTROLS:
 *
 * Gamepad 1:
 *   Left Stick: Drive
 *   Right Stick X: Rotate
 *   A: Slow mode
 *   B: Fast mode
 *
 * Gamepad 2:
 *   A: Set BLUE alliance
 *   B: Set RED alliance
 *   LEFT BUMPER: Start turret aiming (hold to keep aiming)
 *   RIGHT BUMPER: Enable Limelight correction
 *   X: Go to home position
 *   Back: Stop aiming
 *
 * SETUP CHECKLIST:
 * 1. Set goal positions in TurretAiming.java (RED_GOAL, BLUE_GOAL)
 * 2. Set turret angle limits (TURRET_MIN_ANGLE_RAD, TURRET_MAX_ANGLE_RAD)
 * 3. Calibrate TURRET_TICKS_PER_RADIAN
 * 4. Test!
 */
@TeleOp(name = "STEP 1 - Turret Aiming Test", group = "TESTING")
public class Step1_TurretTest extends LinearOpMode {

    // ==================== HARDWARE ====================
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private IMU imu;
    private DcMotorEx turretMotor;
    private Limelight3A limelight;
    private Follower follower;  // For Pinpoint odometry

    // ==================== TURRET AIMING ====================
    private TurretAiming turretAiming;

    // ==================== DRIVE STATE ====================
    private double maxSpeed = 0.77;
    private boolean fieldOriented = false;
    private boolean lastFieldToggle = false;

    // Button debouncing
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();

        // Create turret aiming controller
        turretAiming = new TurretAiming(turretMotor, limelight);

        // Set default alliance
        turretAiming.setAlliance(TurretAiming.Alliance.BLUE);

        telemetry.addLine("════════════════════════════════");
        telemetry.addLine("  STEP 1: TURRET AIMING TEST");
        telemetry.addLine("════════════════════════════════");
        telemetry.addLine();
        telemetry.addLine("This tests ONLY turret aiming");
        telemetry.addLine("No shooting or intake yet!");
        telemetry.addLine();
        telemetry.addLine("LEFT BUMPER: Aim at goal");
        telemetry.addLine("RIGHT BUMPER: Use Limelight");
        telemetry.addLine("A/B: Set alliance");
        telemetry.addLine();
        telemetry.addLine("Press START");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ===== UPDATE ODOMETRY =====
            follower.update();
            Pose robotPose = follower.getPose();
            double robotHeading = robotPose.getHeading();

            // ===== ALLIANCE SELECTION =====
            if (gamepad2.a) {
                turretAiming.setAlliance(TurretAiming.Alliance.BLUE);
                telemetry.speak("Blue alliance");
            }
            if (gamepad2.b) {
                turretAiming.setAlliance(TurretAiming.Alliance.RED);
                telemetry.speak("Red alliance");
            }

            // ===== TURRET AIMING CONTROLS =====

            // LEFT BUMPER: Start/continue aiming
            if (gamepad2.left_bumper && !lastLeftBumper) {
                turretAiming.toggleAiming();
                if (turretAiming.isAiming()) {
                    telemetry.speak("Aiming");
                } else {
                    telemetry.speak("Stopped");
                }
            }
            lastLeftBumper = gamepad2.left_bumper;

            // RIGHT BUMPER: Toggle Limelight correction
            if (gamepad2.right_bumper && !lastRightBumper) {
                turretAiming.toggleLimelightCorrection();
                if (turretAiming.isUsingLimelight()) {
                    telemetry.speak("Limelight on");
                } else {
                    telemetry.speak("Limelight off");
                }
            }
            lastRightBumper = gamepad2.right_bumper;

            // X: Go home
            if (gamepad2.x) {
                turretAiming.goHome();
                telemetry.speak("Going home");
            }

            // BACK: Stop aiming
            if (gamepad2.back) {
                turretAiming.stopAiming();
            }

            // ===== UPDATE TURRET =====
            turretAiming.update(robotPose, robotHeading);

            // ===== DRIVE =====
            handleDrive();

            // ===== TELEMETRY =====
            displayTelemetry(robotPose);
        }
    }

    // ==================== HARDWARE INITIALIZATION ====================

    private void initializeHardware() {
        // Drive motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // IMU for field-oriented drive
        imu = hardwareMap.get(IMU.class, "imu");

        // Turret motor
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Limelight (optional - can be null)
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(8);
            limelight.start();
        } catch (Exception e) {
            limelight = null;
            telemetry.addLine("WARNING: Limelight not found");
        }

        // Pinpoint odometry through Pedro Pathing
        follower = teleConstants.createFollower(hardwareMap, telemetry);
        follower.setStartingPose(new Pose(72, 72, 0));  // Set to your starting position
    }

    // ==================== DRIVE CONTROL ====================

    private void handleDrive() {
        // Speed control
        if (gamepad1.a) maxSpeed = 0.6;
        if (gamepad1.b) maxSpeed = 0.77;

        // Get inputs
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        // Field-oriented drive (optional)
        if (fieldOriented) {
            double heading = imu.getRobotYawPitchRollAngles().getYaw();
            double temp = forward * Math.cos(heading) + right * Math.sin(heading);
            right = -forward * Math.sin(heading) + right * Math.cos(heading);
            forward = temp;
        }

        // Mecanum drive calculation
        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double bl = forward - right + rotate;
        double br = forward + right - rotate;

        // Normalize
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max; fr /= max; bl /= max; br /= max;
        }

        // Apply power
        frontLeftDrive.setPower(fl * maxSpeed);
        frontRightDrive.setPower(fr * maxSpeed);
        backLeftDrive.setPower(bl * maxSpeed);
        backRightDrive.setPower(br * maxSpeed);
    }

    // ==================== TELEMETRY ====================

    private void displayTelemetry(Pose robotPose) {
        Pose goal = turretAiming.getCurrentGoal();

        telemetry.addLine("════════════════════════════════");
        telemetry.addLine("  TURRET AIMING TEST");
        telemetry.addLine("════════════════════════════════");
        telemetry.addLine();

        // Alliance & Goal
        telemetry.addData("Alliance",
                turretAiming.getCurrentGoal() == goal ? "SET" : "ERROR");
        telemetry.addData("Goal Position", "X:%.1f Y:%.1f",
                goal.getX(), goal.getY());
        telemetry.addLine();

        // Turret Status
        telemetry.addLine("─── TURRET STATUS ───");
        telemetry.addData("Aiming", turretAiming.isAiming() ? "YES" : "NO");
        telemetry.addData("Status", turretAiming.getStatusString());
        telemetry.addData("Control", turretAiming.getControlMethodString());
        telemetry.addData("At Target", turretAiming.isAtTarget() ? "✓" : "X");
        telemetry.addData("Angle Safe", turretAiming.isTargetAngleSafe() ? "✓" : "⚠ UNSAFE");
        telemetry.addLine();

        // Turret Position
        telemetry.addLine("─── TURRET POSITION ───");
        telemetry.addData("Current Ticks", turretAiming.getCurrentTicks());
        telemetry.addData("Target Ticks", turretAiming.getTargetTicks());
        telemetry.addData("Error", turretAiming.getError());
        telemetry.addData("Target Angle", "%.1f°", Math.toDegrees(turretAiming.getTargetAngle()));
        telemetry.addData("Current Angle", "%.1f°", Math.toDegrees(turretAiming.getCurrentAngle()));
        telemetry.addLine();

        // Robot Position
        telemetry.addLine("─── ROBOT POSITION ───");
        telemetry.addData("X", "%.1f\"", robotPose.getX());
        telemetry.addData("Y", "%.1f\"", robotPose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(robotPose.getHeading()));

        // Distance to goal
        double dx = goal.getX() - robotPose.getX();
        double dy = goal.getY() - robotPose.getY();
        double distance = Math.sqrt(dx * dx + dy * dy);
        telemetry.addData("Distance to Goal", "%.1f\"", distance);
        telemetry.addLine();

        // Controls reminder
        telemetry.addLine("─── CONTROLS ───");
        telemetry.addLine("L Bumper: Toggle Aim");
        telemetry.addLine("R Bumper: Toggle Limelight");
        telemetry.addLine("X: Go Home");
        telemetry.addLine("A/B: Set Alliance");

        telemetry.update();
    }
}

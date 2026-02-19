package org.firstinspires.ftc.teamcode.testing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooterConstants.GoalManager;

/**
 * CALIBRATION TELEOP
 *
 * Use this to calibrate your shooter system.
 * This does NOT use any shooting calculations - it's for collecting data!
 *
 * CONTROLS:
 * Gamepad1:
 * - Left stick: Drive
 * - Right stick X: Rotate
 * - D-pad Up/Down: Adjust RPM (±50)
 * - D-pad Left/Right: Adjust Hood (±0.01)
 *
 * Gamepad2:
 * - A: Set alliance to BLUE
 * - B: Set alliance to RED
 * - X: Shoot and record data
 * - Y: Reset to defaults
 * - Right trigger: Run intake
 * - Left bumper: Toggle measurement mode
 *
 * HOW TO CALIBRATE:
 * 1. Set alliance color (A for blue, B for red)
 * 2. Drive to a known position on field
 * 3. Adjust RPM and hood position with D-pad
 * 4. Press X to shoot
 * 5. Measure where ball lands
 * 6. Note: Position, Distance, RPM, Hood Position, Actual Range
 * 7. Repeat from 10-15 different positions
 * 8. Use this data to update PhysicsShooter.velocityToRPM()
 */
@TeleOp(name = "⚙ CALIBRATION - Shooter Tuning", group = "CALIBRATION")
public class CalibrationTeleOp extends LinearOpMode {

    /* ================= HARDWARE ================= */
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private DcMotorEx shooterMotor;
    private DcMotorEx intakeMotor;
    private DcMotorEx turretMotor;
    private Servo hoodServo;
    private Servo turretStopperServo;

    private Follower follower;

    /* ================= CALIBRATION STATE ================= */
    private double manualRPM = 1300.0;
    private double manualHoodPosition = 0.157;
    private boolean measurementMode = false;
    private int shotCount = 0;

    private ElapsedTime shootTimer = new ElapsedTime();
    private boolean isShooting = false;

    /* ================= CONSTANTS ================= */
    private static final double SHOOTER_kP = 180.0;
    private static final double SHOOTER_kI = 13.0;
    private static final double SHOOTER_kD = 0.0;
    private static final double SHOOTER_kF = 18.0;
    private static final double INTAKE_VELOCITY = 1800.0;
    private static final double TURRET_HOME = 0.45;
    private static final double TURRET_ACTIVE = 0.2;

    private static final double RPM_INCREMENT = 50.0;
    private static final double HOOD_INCREMENT = 0.01;

    @Override
    public void runOpMode() {
        /* ---------- Hardware Init ---------- */
        initializeHardware();

        /* ---------- Set default alliance ---------- */
        GoalManager.setAlliance(GoalManager.Alliance.BLUE);

        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("  CALIBRATION MODE");
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine();
        telemetry.addLine("Read instructions in code!");
        telemetry.addLine();
        telemetry.addLine("Press START when ready");
        telemetry.update();

        waitForStart();

        /* ================= MAIN LOOP ================= */
        while (opModeIsActive()) {
            // Update odometry
            follower.update();
            Pose robotPose = follower.getPose();
            Vector robotVelocity = follower.getVelocity();

            // Calculate current distance to goal
            double distanceToGoal = GoalManager.getDistanceToGoal(robotPose);
            double velocityMagnitude = Math.sqrt(
                    robotVelocity.getXComponent() * robotVelocity.getXComponent() +
                            robotVelocity.getYComponent() * robotVelocity.getYComponent()
            );

            /* ---------- ALLIANCE SELECTION ---------- */
            if (gamepad2.a) {
                GoalManager.setAlliance(GoalManager.Alliance.BLUE);
            }
            if (gamepad2.b) {
                GoalManager.setAlliance(GoalManager.Alliance.RED);
            }

            /* ---------- RPM ADJUSTMENT ---------- */
            if (gamepad1.dpad_up) {
                manualRPM += RPM_INCREMENT;
                sleep(150);
            }
            if (gamepad1.dpad_down) {
                manualRPM -= RPM_INCREMENT;
                manualRPM = Math.max(0, manualRPM);
                sleep(150);
            }

            /* ---------- HOOD ADJUSTMENT ---------- */
            if (gamepad1.dpad_right) {
                manualHoodPosition += HOOD_INCREMENT;
                manualHoodPosition = Math.min(1.0, manualHoodPosition);
                sleep(150);
            }
            if (gamepad1.dpad_left) {
                manualHoodPosition -= HOOD_INCREMENT;
                manualHoodPosition = Math.max(0.0, manualHoodPosition);
                sleep(150);
            }

            /* ---------- APPLY SETTINGS ---------- */
            shooterMotor.setVelocity(manualRPM);
            hoodServo.setPosition(manualHoodPosition);

            /* ---------- SHOOT ---------- */
            if (gamepad2.x && !isShooting) {
                startShot();
            }

            // Handle shooting sequence
            if (isShooting) {
                if (shootTimer.seconds() > 0.3) {
                    turretStopperServo.setPosition(TURRET_ACTIVE);
                    intakeMotor.setVelocity(-INTAKE_VELOCITY);
                }

                if (shootTimer.seconds() > 2.0) {
                    endShot();
                    shotCount++;

                    // Print calibration point
                    printCalibrationPoint(robotPose, distanceToGoal);
                }
            }

            /* ---------- RESET ---------- */
            if (gamepad2.y) {
                manualRPM = 1300.0;
                manualHoodPosition = 0.157;
            }

            /* ---------- MEASUREMENT MODE ---------- */
            if (gamepad2.left_bumper) {
                measurementMode = !measurementMode;
                sleep(200);
            }

            /* ---------- INTAKE ---------- */
            if (gamepad2.right_trigger > 0.5 && !isShooting) {
                intakeMotor.setVelocity(INTAKE_VELOCITY);
            } else if (!isShooting) {
                intakeMotor.setVelocity(0);
            }

            /* ---------- DRIVE ---------- */
            double forward = -gamepad1.left_stick_y * 0.6;
            double right = gamepad1.left_stick_x * 0.6;
            double rotate = gamepad1.right_stick_x * 0.6;
            drive(forward, right, rotate);

            /* ---------- TELEMETRY ---------- */
            telemetry.addLine("═══════════════════════════════");
            telemetry.addLine("  CALIBRATION MODE");
            telemetry.addLine("═══════════════════════════════");
            telemetry.addData("Alliance", GoalManager.getCurrentAlliance());
            telemetry.addData("Shots Taken", shotCount);
            telemetry.addLine();

            telemetry.addLine("─── CURRENT SETTINGS ───");
            telemetry.addData("RPM (D-pad ↑↓)", "%.0f", manualRPM);
            telemetry.addData("Hood (D-pad ←→)", "%.3f", manualHoodPosition);
            telemetry.addData("Actual RPM", "%.0f", shooterMotor.getVelocity());
            telemetry.addLine();

            telemetry.addLine("─── POSITION ───");
            telemetry.addData("Robot X", "%.1f", robotPose.getX());
            telemetry.addData("Robot Y", "%.1f", robotPose.getY());
            telemetry.addData("Distance to Goal", "%.1f", distanceToGoal);
            telemetry.addData("Velocity", "%.1f in/s", velocityMagnitude);
            telemetry.addLine();

            telemetry.addLine("─── CONTROLS ───");
            telemetry.addLine("X: Shoot & Record");
            telemetry.addLine("Y: Reset Settings");
            telemetry.addLine("A/B: Set Alliance");
            telemetry.addLine();

            if (measurementMode) {
                telemetry.addLine("─── MEASURE SHOT ───");
                telemetry.addLine("After shooting:");
                telemetry.addLine("1. Measure actual range");
                telemetry.addLine("2. Record all values above");
                telemetry.addLine("3. Add to calibration data");
            }

            telemetry.update();
        }
    }

    /* ================= HELPER METHODS ================= */

    private void initializeHardware() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        hoodServo = hardwareMap.get(Servo.class, "turret_servo");
        turretStopperServo = hardwareMap.get(Servo.class, "turretStopper");

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setVelocityPIDFCoefficients(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretStopperServo.setPosition(TURRET_HOME);

        follower = Constants.createFollower(hardwareMap, telemetry);
        follower.setStartingPose(new Pose(72, 72, 0));
    }

    private void startShot() {
        isShooting = true;
        shootTimer.reset();
        turretStopperServo.setPosition(TURRET_HOME);
    }

    private void endShot() {
        isShooting = false;
        turretStopperServo.setPosition(TURRET_HOME);
        intakeMotor.setVelocity(0);
    }

    private void printCalibrationPoint(Pose robotPose, double distance) {
        telemetry.log().add("═══════════════════════════════");
        telemetry.log().add("CALIBRATION POINT #" + shotCount);
        telemetry.log().add("═══════════════════════════════");
        telemetry.log().add(String.format("Position: (%.1f, %.1f)",
                robotPose.getX(), robotPose.getY()));
        telemetry.log().add(String.format("Distance: %.1f inches", distance));
        telemetry.log().add(String.format("RPM: %.0f", manualRPM));
        telemetry.log().add(String.format("Hood: %.3f", manualHoodPosition));
        telemetry.log().add("NOW MEASURE ACTUAL RANGE!");
        telemetry.log().add("Record this data point!");
        telemetry.log().add("═══════════════════════════════");
    }

    private void drive(double forward, double right, double rotate) {
        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double bl = forward - right + rotate;
        double br = forward + right - rotate;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max; fr /= max; bl /= max; br /= max;
        }

        frontLeftDrive.setPower(fl);
        frontRightDrive.setPower(fr);
        backLeftDrive.setPower(bl);
        backRightDrive.setPower(br);
    }
}

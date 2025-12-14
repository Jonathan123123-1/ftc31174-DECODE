package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "100% - FullTeleOpBlue", group = "Robot")
public class FullTeleOpBlue extends LinearOpMode {

    // --- Drive Motors ---
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    // --- IMU ---
    private IMU imu;

    // --- Speed Control ---
    private double maxSpeed = 1.0;

    // --- Field-oriented toggle ---
    private boolean fieldOriented = false;
    private boolean lastTogglePressed = false;

    // --- Shooter Motor ---
    private DcMotorEx shooterMotor;
    private double SHOOTER_POWER = 0.58;
    private boolean shooterOn = false;
    private boolean lastTriggerPressed = false;

    // --- Intake Motor ---
    private DcMotorEx intakeMotor;
    private double INTAKE_POWER = 5;
    private boolean timedIntakeActive = false;
    private long intakeStartTime = 0;
    private final long TIMED_INTAKE_DURATION_MS = 400;

    // --- Roller Motor ---
    private DcMotorEx rollerMotor;
    private double ROLLER_POWER = 1;

    private ElapsedTime timer = new ElapsedTime();

    // --- Vision ---
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    private final int blueGoalID = 20; // Blue alliance tag
    private boolean cameraActive = false;
    private boolean lastLeftBumper = false;

    @Override
    public void runOpMode() {

        // --- Drive Motors ---
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- IMU ---
        imu = hardwareMap.get(IMU.class, "imu");

        // --- Shooter ---
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // --- Intake ---
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Roller ---
        rollerMotor = hardwareMap.get(DcMotorEx.class, "rollerMotor");
        rollerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rollerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Vision ---
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(tagProcessor)
                .build();

        telemetry.addLine("FullTeleOpBlue Initialized");
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            long currentTime = System.currentTimeMillis();

            // -------------------------
            // DRIVER 1 — TOGGLE FIELD ORIENTED
            // -------------------------
            boolean togglePressed = gamepad1.x;
            if (togglePressed && !lastTogglePressed) fieldOriented = !fieldOriented;
            lastTogglePressed = togglePressed;

            // -------------------------
            // DRIVER 1 — TOGGLE CAMERA
            // -------------------------
            if (gamepad1.left_bumper && !lastLeftBumper) cameraActive = !cameraActive;
            lastLeftBumper = gamepad1.left_bumper;

            // -------------------------
            // DRIVER 1 — BASE MOVEMENT
            // -------------------------
            double forward = -gamepad1.left_stick_y;
            double right = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            if (gamepad1.a) maxSpeed = 0.5;
            if (gamepad1.b) maxSpeed = 1;

            // -------------------------
            // VISION PROCESSING
            // -------------------------
            List<AprilTagDetection> detections = null;
            AprilTagDetection targetTag = null;

            if (cameraActive) {
                detections = tagProcessor.getDetections();
                for (AprilTagDetection tag : detections) {
                    if (tag.id == blueGoalID) {
                        targetTag = tag;
                        break;
                    }
                }

                telemetry.addLine("Camera ACTIVE");
                telemetry.addData("Camera State", visionPortal.getCameraState());

                if (targetTag != null && targetTag.ftcPose != null) {
                    double tagCenterX = targetTag.center.x;
                    double camCenterX = 320;

                    double xOffset = tagCenterX - camCenterX;
                    double normalizedX = xOffset / camCenterX;

                    // -------------------------
                    // RIGHT BUMPER → AUTO AIM
                    // -------------------------
                    if (gamepad1.right_bumper) {
                        double Kp = 0.5;
                        double minPower = 0.2;
                        double maxPower = 0.5; // max strafing power

                        // Only strafe left/right based on camera
                        right = normalizedX * Kp;

                        // Ensure minimum power for small offsets
                        if (Math.abs(right) < minPower && Math.abs(normalizedX) > 0.05)
                            right = Math.signum(right) * minPower;

                        // Cap the strafing so it doesn't go too fast
                        if (Math.abs(right) > maxPower)
                            right = Math.signum(right) * maxPower;

                        forward = 0; // no forward/backward movement
                        telemetry.addLine("AUTO AIM ACTIVE");
                    }
                }
            }

            // -------------------------
            // FIELD-ORIENTED ADJUSTMENT
            // -------------------------
            if (fieldOriented) {
                double heading = imu.getRobotYawPitchRollAngles().getYaw();
                double temp = forward * Math.cos(heading) + right * Math.sin(heading);
                right = -forward * Math.sin(heading) + right * Math.cos(heading);
                forward = temp;
            }

            // -------------------------
            // DRIVE
            // -------------------------
            drive(forward, right, rotate);

            // -------------------------
            // OPERATOR 2 — SHOOTER
            // -------------------------
            boolean triggerPressed = gamepad2.right_trigger > 0.5;
            if (triggerPressed && !lastTriggerPressed) shooterOn = !shooterOn;
            lastTriggerPressed = triggerPressed;

            // Left trigger increases shooter motor speed
            double currentShooterPower = SHOOTER_POWER;
            if (gamepad2.left_trigger > 0.5) {
                currentShooterPower = 0.85; // higher power for farther shots
            }

            shooterMotor.setPower(shooterOn ? currentShooterPower : 0);

            // -------------------------
            // OPERATOR 2 — TIMED INTAKE
            // -------------------------
            if (gamepad2.left_bumper && !timedIntakeActive) {
                timedIntakeActive = true;
                intakeStartTime = currentTime;
                intakeMotor.setPower(-INTAKE_POWER);
            }

            if (timedIntakeActive) {
                if (currentTime - intakeStartTime >= TIMED_INTAKE_DURATION_MS) {
                    intakeMotor.setPower(0);
                    timedIntakeActive = false;
                }
            } else {
                if (gamepad2.dpad_up) intakeMotor.setPower(-INTAKE_POWER);
                else if (gamepad2.dpad_down) intakeMotor.setPower(INTAKE_POWER);
                else if (gamepad2.dpad_left || gamepad2.dpad_right) intakeMotor.setPower(0);
            }

            // -------------------------
            // OPERATOR 2 — ROLLER CONTROL
            // -------------------------
            if (gamepad2.y) rollerMotor.setPower(-ROLLER_POWER);
            else if (gamepad2.a) rollerMotor.setPower(ROLLER_POWER);
            else if (gamepad2.x || gamepad2.b) rollerMotor.setPower(0);

            // -------------------------
            // TELEMETRY
            // -------------------------
            telemetry.addData("Speed Mode", maxSpeed);
            telemetry.addData("Field Oriented", fieldOriented ? "ON" : "OFF");
            telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
            telemetry.addData("Shooter Power", currentShooterPower);
            telemetry.addData("Intake Power", intakeMotor.getPower());
            telemetry.addData("Timed Intake", timedIntakeActive);
            telemetry.addData("Roller Power", rollerMotor.getPower());
            telemetry.addData("FL Power", frontLeftDrive.getPower());
            telemetry.addData("FR Power", frontRightDrive.getPower());
            telemetry.addData("BL Power", backLeftDrive.getPower());
            telemetry.addData("BR Power", backRightDrive.getPower());
            telemetry.update();
        }
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

        frontLeftDrive.setPower(fl * maxSpeed);
        frontRightDrive.setPower(fr * maxSpeed);
        backLeftDrive.setPower(bl * maxSpeed);
        backRightDrive.setPower(br * maxSpeed);
    }
}
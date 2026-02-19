package org.firstinspires.ftc.teamcode.Auto;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "100% - FullAuto Blue (Meet)", group = "Autonomous")
public class AutoBlueTest extends LinearOpMode {

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private DcMotor shooterMotor;
    private DcMotor intakeMotor;
    private DcMotor rollerMotor;

    // ===== POWERS (MATCH TELEOP BLUE) =====
    private double SHOOTER_POWER = 0.6;   // same as TeleOp
    private double INTAKE_POWER  = 5.0;   // same as TeleOp
    private double ROLLER_POWER  = 1.0;   // same as TeleOp   // negative = pull game piece in

    // Motor constants
    private final double TICKS_PER_REV = 537.6; // REV HD 40:1
    private final double WHEEL_DIAMETER_INCH = 4.0;
    private final double COUNTS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCH);
    private final double DRIVE_POWER = 0.4;

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    private final int blueGoalID = 20;

    @Override
    public void runOpMode() {

        // --- Hardware ---
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        shooterMotor = hardwareMap.get(DcMotor.class, "shooter_motor");
        intakeMotor  = hardwareMap.get(DcMotor.class, "intake_motor");
        rollerMotor  = hardwareMap.get(DcMotor.class, "rollerMotor");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rollerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- Vision setup ---
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

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        // ===== START SHOOTER BEFORE MOVING =====
        shooterMotor.setPower(SHOOTER_POWER);
        sleep(3000); // spin-up time

        // --- Step 1: Move backward 48 inches with camera correction ---
        moveBackwardWithCamera(33);

        // ===== AFTER BACKING UP: FEED SEQUENCE =====
        runFeedCycle(1.0, 0.4);

        // stop shooter after cycles
        shooterMotor.setPower(0);

        sleep(300);

        // --- Step 2: Strafe 24 inches left ---
        strafeLeft(28);

        telemetry.addLine("Autonomous routine complete");
        telemetry.update();
        sleep(1000);
    }

    private void runFeedCycle(double shooterTimeSec, double intakeTimeSec) {
        // Matches TeleOp logic:
        // roller (-) -> intake (+) -> roller (-) -> intake (+) -> roller (-)

        // wait while shooter is up to speed
        sleep((long)(shooterTimeSec * 1500));

        // 1) roller reverse (feed)
        rollerMotor.setPower(-ROLLER_POWER);
        intakeMotor.setPower(-INTAKE_POWER);
        sleep((long)(shooterTimeSec * 2000));
        rollerMotor.setPower(0);

        // 2) intake forward
        intakeMotor.setPower(INTAKE_POWER);
        sleep((long)(intakeTimeSec * 1000));
        intakeMotor.setPower(0);

        // 3) roller reverse again
        rollerMotor.setPower(-ROLLER_POWER);
        intakeMotor.setPower(-INTAKE_POWER);
        sleep((long)(shooterTimeSec * 2000));
        rollerMotor.setPower(0);

        // 4) intake forward again
        intakeMotor.setPower(INTAKE_POWER);
        sleep((long)(intakeTimeSec * 1000));
        intakeMotor.setPower(0);

        // 5) final feed: roller + intake SAME TIME
        rollerMotor.setPower(-ROLLER_POWER);
        intakeMotor.setPower(-INTAKE_POWER);
        sleep((long)(1.0 * 2000)); // both run together for 1 second
        rollerMotor.setPower(0);
        intakeMotor.setPower(0);
    }

    private void moveBackwardWithCamera(double inches) {
        int moveCounts = (int)(inches * COUNTS_PER_INCH);

        frontLeftDrive.setTargetPosition(-moveCounts);
        frontRightDrive.setTargetPosition(-moveCounts);
        backLeftDrive.setTargetPosition(-moveCounts);
        backRightDrive.setTargetPosition(-moveCounts);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double Kp = 0.375;

        while (opModeIsActive() &&
                (frontLeftDrive.isBusy() || frontRightDrive.isBusy() ||
                        backLeftDrive.isBusy() || backRightDrive.isBusy())) {

            List<AprilTagDetection> detections = tagProcessor.getDetections();
            AprilTagDetection targetTag = null;

            for (AprilTagDetection tag : detections) {
                if (tag.id == blueGoalID) {
                    targetTag = tag;
                    break;
                }
            }

            double strafe = 0;

            if (targetTag != null && targetTag.ftcPose != null) {
                double tagCenterX = targetTag.center.x;
                double camCenterX = 320;
                double normalizedX = (tagCenterX - camCenterX) / camCenterX;
                strafe = Kp * normalizedX;
                strafe = Math.max(-0.3, Math.min(0.3, strafe));
            }

            frontLeftDrive.setPower(-DRIVE_POWER + strafe);
            frontRightDrive.setPower(-DRIVE_POWER - strafe);
            backLeftDrive.setPower(-DRIVE_POWER - strafe);
            backRightDrive.setPower(-DRIVE_POWER + strafe);
        }

        stopMotors();
    }

    private void strafeLeft(double inches) {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int moveCounts = (int)(inches * COUNTS_PER_INCH);

        frontLeftDrive.setTargetPosition(-moveCounts);
        frontRightDrive.setTargetPosition(moveCounts);
        backLeftDrive.setTargetPosition(moveCounts);
        backRightDrive.setTargetPosition(-moveCounts);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftDrive.setPower(0.5);
        frontRightDrive.setPower(0.5);
        backLeftDrive.setPower(0.5);
        backRightDrive.setPower(0.5);

        while (opModeIsActive() &&
                (frontLeftDrive.isBusy() || frontRightDrive.isBusy() ||
                        backLeftDrive.isBusy() || backRightDrive.isBusy())) {}

        stopMotors();
    }

    private void stopMotors() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}

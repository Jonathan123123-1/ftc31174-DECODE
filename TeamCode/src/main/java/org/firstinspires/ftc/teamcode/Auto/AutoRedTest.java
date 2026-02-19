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

@Autonomous(name = "100% - FullAuto Red (Meet)", group = "Autonomous")
public class AutoRedTest extends LinearOpMode {

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private DcMotor shooterMotor;
    private DcMotor intakeMotor;
    private DcMotor rollerMotor;

    // ===== POWERS (MATCH TELEOP RED) =====
    private double SHOOTER_POWER = 0.6;
    private double INTAKE_POWER  = 5.0;
    private double ROLLER_POWER  = 1.0; // negative = pull game piece in

    private final double TICKS_PER_REV = 537.6;
    private final double WHEEL_DIAMETER_INCH = 4.0;
    private final double COUNTS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCH);
    private final double DRIVE_POWER = 0.4;

    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    private final int redGoalID = 24;

    @Override
    public void runOpMode() {

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

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        shooterMotor.setPower(SHOOTER_POWER);
        sleep(3000);

        moveBackwardWithCamera(33);

        runFeedCycle(1.0, 0.4);

        shooterMotor.setPower(0);

        sleep(300);

        strafeRight(28);

        telemetry.addLine("Autonomous routine complete");
        telemetry.update();
        sleep(1000);
    }

    private void runFeedCycle(double shooterTimeSec, double intakeTimeSec) {
        sleep((long)(shooterTimeSec * 1500));

        rollerMotor.setPower(-ROLLER_POWER);
        intakeMotor.setPower(-INTAKE_POWER);
        sleep((long)(shooterTimeSec * 2000));
        rollerMotor.setPower(0);

        intakeMotor.setPower(INTAKE_POWER);
        sleep((long)(intakeTimeSec * 1000));
        intakeMotor.setPower(0);

        rollerMotor.setPower(-ROLLER_POWER);
        intakeMotor.setPower(-INTAKE_POWER);
        sleep((long)(shooterTimeSec * 2000));
        rollerMotor.setPower(0);

        intakeMotor.setPower(INTAKE_POWER);
        sleep((long)(intakeTimeSec * 1000));
        intakeMotor.setPower(0);

        rollerMotor.setPower(-ROLLER_POWER);
        intakeMotor.setPower(-INTAKE_POWER);
        sleep(2000);
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
                if (tag.id == redGoalID) {
                    targetTag = tag;
                    break;
                }
            }

            double strafe = 0;

            if (targetTag != null && targetTag.ftcPose != null) {
                double normalizedX = (targetTag.center.x - 320) / 320;
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

    private void strafeRight(double inches) {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int moveCounts = (int)(inches * COUNTS_PER_INCH);

        frontLeftDrive.setTargetPosition(moveCounts);
        frontRightDrive.setTargetPosition(-moveCounts);
        backLeftDrive.setTargetPosition(-moveCounts);
        backRightDrive.setTargetPosition(moveCounts);

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
package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "AutoTest", group = "Autonomous")
public class AutoTest extends LinearOpMode {

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    // Motor constants
    private final double TICKS_PER_REV = 537.6; // REV HD 40:1
    private final double WHEEL_DIAMETER_INCH = 4.0; // Inches
    private final double COUNTS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCH);
    private final double DRIVE_POWER = 0.6;

    @Override
    public void runOpMode() {
        // Hardware mapping
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        // Correct directions (adjust if reversed)
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Reset encoders
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        // Calculate target counts for 48 inches backward
        int moveCounts = (int)(48 * COUNTS_PER_INCH);

        // Set target positions (negative for backward)
        frontLeftDrive.setTargetPosition(-moveCounts);
        frontRightDrive.setTargetPosition(-moveCounts);
        backLeftDrive.setTargetPosition(-moveCounts);
        backRightDrive.setTargetPosition(-moveCounts);

        // RUN_TO_POSITION mode
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power
        frontLeftDrive.setPower(DRIVE_POWER);
        frontRightDrive.setPower(DRIVE_POWER);
        backLeftDrive.setPower(DRIVE_POWER);
        backRightDrive.setPower(DRIVE_POWER);

        // Wait until motors reach target
        while (opModeIsActive() &&
                frontLeftDrive.isBusy() &&
                frontRightDrive.isBusy() &&
                backLeftDrive.isBusy() &&
                backRightDrive.isBusy()) {

            telemetry.addData("FL", frontLeftDrive.getCurrentPosition());
            telemetry.addData("FR", frontRightDrive.getCurrentPosition());
            telemetry.addData("BL", backLeftDrive.getCurrentPosition());
            telemetry.addData("BR", backRightDrive.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motors
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

        telemetry.addLine("Done moving 48 inches backward");
        telemetry.update();
        sleep(1000);
    }
}
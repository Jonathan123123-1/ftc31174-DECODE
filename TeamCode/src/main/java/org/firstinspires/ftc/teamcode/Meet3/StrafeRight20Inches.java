package org.firstinspires.ftc.teamcode.Meet3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Strafe Right 20 Inches", group = "TEST")
public class StrafeRight20Inches extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private static final double TICKS_PER_INCH = 45.0;
    private static final int STRAFE_DISTANCE_TICKS = (int)(20 * TICKS_PER_INCH);

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRight = hardwareMap.get(DcMotor.class, "back_right_drive");

        // Reverse left side
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set target positions (STRAFE RIGHT)
        frontLeft.setTargetPosition( STRAFE_DISTANCE_TICKS);
        backLeft.setTargetPosition(-STRAFE_DISTANCE_TICKS);
        frontRight.setTargetPosition(-STRAFE_DISTANCE_TICKS);
        backRight.setTargetPosition( STRAFE_DISTANCE_TICKS);

        // Run to position
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        // Set power
        frontLeft.setPower(0.5);
        backLeft.setPower(0.5);
        frontRight.setPower(0.5);
        backRight.setPower(0.5);

        // Wait until motors finish
        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy()
                        || backLeft.isBusy() || backRight.isBusy())) {
            idle();
        }

        // Stop motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}

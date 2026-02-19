package org.firstinspires.ftc.teamcode.Meet3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Strafe Left 20 Inches", group = "Simple")
public class StrafeLeft20 extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight;

    // ---- TUNABLE CONSTANTS ----
    static final double TICKS_PER_REV = 537.7; // goBILDA 312 RPM
    static final double WHEEL_DIAMETER_INCHES = 3.78;
    static final double STRAFE_MULTIPLIER = 1.1; // strafing correction
    static final double INCHES = 20;

    @Override
    public void runOpMode() {

        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        resetEncoders();

        waitForStart();
        if (isStopRequested()) return;

        int moveTicks = inchesToTicks(INCHES);

        // STRAFE LEFT
        frontLeft.setTargetPosition(-moveTicks);
        frontRight.setTargetPosition(moveTicks);
        backLeft.setTargetPosition(moveTicks);
        backRight.setTargetPosition(-moveTicks);

        setRunToPosition();
        setPower(0.5);

        while (opModeIsActive() && motorsBusy()) {
            idle();
        }

        stopMotors();
    }

    // ---------------- HELPERS ----------------

    private int inchesToTicks(double inches) {
        double circumference = Math.PI * WHEEL_DIAMETER_INCHES;
        return (int) ((inches / circumference) * TICKS_PER_REV * STRAFE_MULTIPLIER);
    }

    private void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setRunToPosition() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void setPower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    private boolean motorsBusy() {
        return frontLeft.isBusy() || frontRight.isBusy() ||
                backLeft.isBusy() || backRight.isBusy();
    }

    private void stopMotors() {
        setPower(0);
    }
}

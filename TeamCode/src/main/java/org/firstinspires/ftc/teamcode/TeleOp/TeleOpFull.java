package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "First - Idk", group = "Robot")
public class TeleOpFull extends OpMode {

    // --- Drive Motors ---
    DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    // --- IMU ---
    IMU imu;

    // --- Speed Control ---
    double maxSpeed = 1.0;

    // --- Shooter Motor ---
    DcMotorEx shooterMotor;
    double SHOOTER_POWER = 0.6;
    boolean shooterOn = false;
    boolean lastTriggerPressed = false;

    // --- Intake Motor ---
    DcMotorEx intakeMotor;
    double INTAKE_POWER = 5;

    // --- Roller Motor ---
    DcMotorEx rollerMotor;
    double ROLLER_POWER = 0.7;

    // --- Timed Intake ---
    boolean timedIntakeActive = false;
    long intakeStartTime = 0;
    final long TIMED_INTAKE_DURATION_MS = 400;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {

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

        // --- Shooter ---
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // --- Intake ---
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Roller motor ---
        rollerMotor = hardwareMap.get(DcMotorEx.class, "rollerMotor");
        rollerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rollerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Dual Controller TeleOp (NO SERVOS) + Roller Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        long currentTime = System.currentTimeMillis();

        // -------------------------
        // DRIVER 1 — MOVEMENT
        // -------------------------
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        if (gamepad1.a) maxSpeed = 0.5;
        if (gamepad1.b) maxSpeed = 1.0;

        drive(forward, right, rotate);

        // -------------------------
        // OPERATOR 2 — SHOOTER
        // -------------------------

        boolean triggerPressed = gamepad2.right_trigger > 0.5;
        if (triggerPressed && !lastTriggerPressed) shooterOn = !shooterOn;
        lastTriggerPressed = triggerPressed;
        shooterMotor.setPower(shooterOn ? SHOOTER_POWER : 0);

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

        if (gamepad2.y) {
            rollerMotor.setPower(ROLLER_POWER);   // forward
        } else if (gamepad2.a) {
            rollerMotor.setPower(-ROLLER_POWER);  // reverse
        } else if (gamepad2.x || gamepad2.b) {
            rollerMotor.setPower(0);              // stop
        }

        // -------------------------
        // TELEMETRY
        // -------------------------
        telemetry.addData("Speed Mode", maxSpeed);
        telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
        telemetry.addData("Intake Power", intakeMotor.getPower());
        telemetry.addData("Timed Intake", timedIntakeActive);
        telemetry.addData("Roller Power", rollerMotor.getPower());
        telemetry.update();
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

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "Movement Only (Field Oriented)", group = "Robot")
public class MovementOnly extends OpMode {

    // --- Drive Motors ---
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    // --- IMU ---
    private IMU imu;

    // --- Speed Control ---
    private double maxSpeed = 0.95;

    // --- Field-oriented toggle ---
    private boolean fieldOriented = false;
    private boolean lastTogglePressed = false;

    @Override
    public void init() {
        // Initialize motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        // Reverse left side
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Also set zero power behavior for instant stop
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");

        telemetry.addLine("Movement Only TeleOp Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Toggle field-oriented mode with gamepad1.x
        boolean togglePressed = gamepad1.x;
        if (togglePressed && !lastTogglePressed) {
            fieldOriented = !fieldOriented;
        }
        lastTogglePressed = togglePressed;

        // Speed modes
        if (gamepad1.a) maxSpeed = 0.5;
        if (gamepad1.b) maxSpeed = 0.95;

        // Driver sticks
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        if (Math.abs(forward) < 0.1) forward = 0;
        if (Math.abs(right) < 0.1) right = 0;
        if (Math.abs(rotate) < 0.1) rotate = 0;

        forward = Math.signum(forward) * forward * forward;
        right = Math.signum(right) * right * right;
        rotate = Math.signum(rotate) * rotate * rotate;

        // Field-oriented transformation
        if (fieldOriented) {
            double heading = imu.getRobotYawPitchRollAngles().getYaw(); // radians
            double temp = forward * Math.cos(heading) + right * Math.sin(heading);
            right = -forward * Math.sin(heading) + right * Math.cos(heading);
            forward = temp;
        }

        // Drive
        drive(forward, right, rotate);


        // Telemetry
        telemetry.addData("Speed Mode", maxSpeed);
        telemetry.addData("Field Oriented", fieldOriented ? "ON" : "OFF");
        telemetry.addData("FL Power", frontLeftDrive.getPower());
        telemetry.addData("FR Power", frontRightDrive.getPower());
        telemetry.addData("BL Power", backLeftDrive.getPower());
        telemetry.addData("BR Power", backRightDrive.getPower());
        telemetry.update();
    }

    private void drive(double forward, double right, double rotate) {
        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double bl = forward - right + rotate;
        double br = forward + right - rotate;

        // Normalize
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        double turboMultiplier = gamepad1.left_bumper ? 1.0 : maxSpeed;

        // Apply speed
        frontLeftDrive.setPower(fl * turboMultiplier);
        frontRightDrive.setPower(fr * turboMultiplier);
        backLeftDrive.setPower(bl * turboMultiplier);
        backRightDrive.setPower(br * turboMultiplier);
    }
}

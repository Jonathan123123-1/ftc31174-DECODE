package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Turret TeleOp", group = "TeleOp")
public class TurretTeleOpSpeed extends LinearOpMode {

    DcMotor turretMotor;
    Servo turretServo;

    double turretPower = 0.0;
    double servoPosition = 0.0;

    boolean lastRB = false;
    boolean lastLB = false;
    boolean lastDPADRight = false;
    boolean lastDPADLeft = false;

    @Override
    public void runOpMode() {

        turretMotor = hardwareMap.get(DcMotor.class, "shooter_motor");
        turretMotor.setDirection(DcMotor.Direction.REVERSE);

        turretServo = hardwareMap.get(Servo.class, "turret_servo");
        turretServo.setPosition(servoPosition); // initialize servo position

        waitForStart();

        while (opModeIsActive()) {

            // === Turret Motor Controls ===
            if (gamepad1.right_bumper && !lastRB) {
                if (turretPower < 1.0) {
                    turretPower += 0.1;
                    if (turretPower > 1.0) turretPower = 1.0;
                } else {
                    turretPower = 0.0; // reset if at max
                }
            }

            if (gamepad1.left_bumper && !lastLB) {
                if (turretPower > 0.0) {
                    turretPower -= 0.1;
                    if (turretPower < 0.0) turretPower = 0.0;
                }
            }

            turretMotor.setPower(turretPower);

            // === Hood Servo Controls ===
            if (gamepad1.dpad_right && !lastDPADRight) {
                if (servoPosition < 1.0) {
                    servoPosition += 0.1;
                    if (servoPosition > 1.0) servoPosition = 1.0;
                } else {
                    servoPosition = 0.0; // reset if at max
                }
            }

            if (gamepad1.dpad_left && !lastDPADLeft) {
                if (servoPosition > 0.0) {
                    servoPosition -= 0.1;
                    if (servoPosition < 0.0) servoPosition = 0.0;
                }
            }

            turretServo.setPosition(servoPosition);

            // === Update last button states ===
            lastRB = gamepad1.right_bumper;
            lastLB = gamepad1.left_bumper;
            lastDPADRight = gamepad1.dpad_right;
            lastDPADLeft = gamepad1.dpad_left;

            // === Telemetry ===
            telemetry.addData("Turret Power", turretPower);
            telemetry.addData("Hood Servo Position", servoPosition);
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Turret TeleOP CLOSE", group = "TeleOp")
public class TurretCloseShoot extends LinearOpMode {

    DcMotorEx turretMotor;
    Servo turretServo;

    // Shooter target
    final double MAX_RPM = 6000;           // motor max RPM
    double targetVelocity = 0.3 * MAX_RPM; // match old 0.7 power

    double servoPosition = 0.7;            // hood starting position

    // PIDF coefficients
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
    double kF = 0.0;

    // Last button states
    boolean lastDPADRight = false;
    boolean lastDPADLeft = false;
    boolean lastA = false;
    boolean lastB = false;
    boolean lastX = false;
    boolean lastY = false;

    @Override
    public void runOpMode() {

        // Initialize shooter motor
        turretMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turretMotor.setDirection(DcMotorEx.Direction.REVERSE);
        turretMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        // Initialize hood servo
        turretServo = hardwareMap.get(Servo.class, "turret_servo");
        turretServo.setPosition(servoPosition);

        waitForStart();

        while (opModeIsActive()) {

            // === Maintain shooter velocity ===
            turretMotor.setVelocity(targetVelocity);

            // === Hood servo controls ===
            if (gamepad1.dpad_right && !lastDPADRight) {
                if (servoPosition < 1.0) {
                    servoPosition += 0.1;
                    if (servoPosition > 1.0) servoPosition = 1.0;
                } else {
                    servoPosition = 0.0;
                }
            }

            if (gamepad1.dpad_left && !lastDPADLeft) {
                if (servoPosition > 0.0) {
                    servoPosition -= 0.1;
                    if (servoPosition < 0.0) servoPosition = 0.0;
                }
            }

            turretServo.setPosition(servoPosition);

            // === PIDF live tuning buttons ===
            if (gamepad1.a && !lastA) {
                kP += 0.1;
                turretMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
            }
            if (gamepad1.b && !lastB) {
                kP -= 0.1;
                if (kP < 0) kP = 0;
                turretMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
            }
            if (gamepad1.x && !lastX) {
                kF += 0.1;
                turretMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
            }
            if (gamepad1.y && !lastY) {
                kF -= 0.1;
                if (kF < 0) kF = 0;
                turretMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
            }

            // === Update last button states ===
            lastDPADRight = gamepad1.dpad_right;
            lastDPADLeft = gamepad1.dpad_left;
            lastA = gamepad1.a;
            lastB = gamepad1.b;
            lastX = gamepad1.x;
            lastY = gamepad1.y;

            // === Telemetry ===
            telemetry.addData("Hood Position", servoPosition);
            telemetry.addData("Target RPM", targetVelocity);
            telemetry.addData("Current RPM", turretMotor.getVelocity());
            telemetry.addData("kP", kP);
            telemetry.addData("kF", kF);
            telemetry.update();
        }
    }
}

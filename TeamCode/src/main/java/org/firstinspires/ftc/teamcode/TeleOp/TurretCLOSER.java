package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Turret CLOSER", group = "TeleOp")
public class TurretCLOSER extends LinearOpMode {

    DcMotorEx turretMotor;
    Servo turretServo;

    // Shooter target
    final double MAX_RPM = 6000;           // motor max RPM
    double targetVelocity = 0.25 * MAX_RPM; // match old 0.7 power

    double servoPosition = 0.5;            // hood starting position

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


        // Initialize hood servo
        turretServo = hardwareMap.get(Servo.class, "turret_servo");
        turretServo.setPosition(servoPosition);

        // Set initial PIDF coefficients
        turretMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        waitForStart();

        while (opModeIsActive()) {

            // === Adjust Target RPM with Triggers ===
            double triggerFactor = 50.0; // Controls how fast the RPM changes
            targetVelocity += gamepad1.right_trigger * triggerFactor;
            targetVelocity -= gamepad1.left_trigger * triggerFactor;

            // Clamp the target velocity to be within the motor's limits
            if (targetVelocity > MAX_RPM) {
                targetVelocity = MAX_RPM;
            }
            if (targetVelocity < 0) {
                targetVelocity = 0;
            }

            // === Maintain shooter velocity ===
            final double TICKS_PER_REV = 28; // adjust for your motor
            double targetTicksPerSec = targetVelocity * TICKS_PER_REV / 60.0;
            turretMotor.setVelocity(-targetTicksPerSec);

            // === Hood servo controls ===
            if (gamepad1.dpad_right && !lastDPADRight) {
                servoPosition += 0.1;
                if (servoPosition > 1.0) servoPosition = 1.0;
            }
            if (gamepad1.dpad_left && !lastDPADLeft) {
                servoPosition -= 0.1;
                if (servoPosition < 0.0) servoPosition = 0.0;
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
            double currentRPM = turretMotor.getVelocity() * 60 / TICKS_PER_REV;
            telemetry.addData("Hood Position", "%.2f", servoPosition);
            telemetry.addData("Target RPM", "%.1f", targetVelocity);
            telemetry.addData("Current RPM", "%.1f", currentRPM);
            telemetry.addData("kP", "%.2f", kP);
            telemetry.addData("kF", "%.2f", kF);
            telemetry.update();
        }
    }
}

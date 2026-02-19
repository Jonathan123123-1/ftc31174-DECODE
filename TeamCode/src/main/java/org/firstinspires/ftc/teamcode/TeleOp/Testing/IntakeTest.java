package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Intake and Stopper Tuner", group = "TEST")
public class IntakeTest extends LinearOpMode {

    // --- Hardware ---
    private DcMotorEx intakeMotor;
    private Servo turretStopper;

    // --- Velocity Tuning ---
    private double targetVelocity = 3000; // Starting velocity, now mutable

    // --- Control Logic ---
    private boolean lastDpadRightPressed = false;
    private boolean lastBumperLeft2 = false;
    private boolean lastBumperRight2 = false;

    @Override
    public void runOpMode() {

        // --- Initialization ---
        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
            turretStopper = hardwareMap.get(Servo.class, "turretStopper");

            PIDFCoefficients pidfCoefficients = intakeMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            double new_P = 40.0;
            double new_F = 20.0;
            PIDFCoefficients newPIDF = new PIDFCoefficients(new_P, pidfCoefficients.i, pidfCoefficients.d, new_F);
            intakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);
            intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            turretStopper.setDirection(Servo.Direction.REVERSE);
            turretStopper.setPosition(0.5);

        } catch (Exception e) {
            telemetry.addLine("ERROR: Could not find or cast a required device.");
            telemetry.addData("Error Msg", e.getMessage());
            telemetry.update();
            sleep(10000);
            return;
        }

        telemetry.addLine("Intake and Stopper Tuner Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- Gamepad 2 Controls ---
            // Intake Velocity Adjustment
            boolean bumperLeft = gamepad2.left_bumper;
            if (bumperLeft && !lastBumperLeft2) {
                targetVelocity -= 400;
            }
            lastBumperLeft2 = bumperLeft;

            boolean bumperRight = gamepad2.right_bumper;
            if (bumperRight && !lastBumperRight2) {
                targetVelocity += 100;
            }
            lastBumperRight2 = bumperRight;

            // Intake Motor Control
            if (gamepad2.dpad_up) intakeMotor.setVelocity(targetVelocity);
            else if (gamepad2.dpad_down) intakeMotor.setVelocity(-targetVelocity);
            else if (gamepad2.dpad_left) intakeMotor.setPower(0.0);

            // Stopper
            boolean dpadRightPressed = gamepad2.dpad_right;
            if (dpadRightPressed && !lastDpadRightPressed) {
                turretStopper.setPosition(0);
                sleep(200);
                turretStopper.setPosition(0.5);
            }
            lastDpadRightPressed = dpadRightPressed;

            // --- Telemetry ---
            telemetry.addData("Current Target Velocity", targetVelocity);
            telemetry.addData("Actual Intake Velocity", intakeMotor.getVelocity());
            telemetry.update();
        }
    }
}

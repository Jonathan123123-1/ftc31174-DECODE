package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Shooter and Intake Test", group = "TEST")
public class ShooterStopperTest extends LinearOpMode {

    // --- Hardware ---
    private DcMotorEx shooterMotor;
    private DcMotorEx intakeMotor; // Replaced Servo with DcMotor

    // --- Control Logic ---
    private boolean shooterOn = false;
    private boolean lastTriggerPressed = false;

    private final double SHOOTER_POWER = 0.6;
    private final double INTAKE_VELOCITY = 1800; // Velocity for the intake motor

    @Override
    public void runOpMode() {

        // --- Initialization ---
        try {
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor"); // Hardware map the intake motor

            shooterMotor.setDirection(DcMotor.Direction.FORWARD);
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            intakeMotor.setDirection(DcMotor.Direction.FORWARD); // Set direction, might need to be REVERSE
            intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Changed for velocity control
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        } catch (Exception e) {
            telemetry.addLine("ERROR: Could not find 'shooter_motor' or 'intake_motor'.");
            telemetry.addLine(e.getMessage());
            telemetry.update();
            sleep(10000);
            return;
        }

        telemetry.addLine("Shooter & Intake Test Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- Gamepad 2 Controls ---

            // --- Right Trigger: Toggle Shooter Motor ---
            boolean triggerPressed = gamepad2.right_trigger > 0.5;
            if (triggerPressed && !lastTriggerPressed) {
                shooterOn = !shooterOn; // Flip the state
            }
            lastTriggerPressed = triggerPressed;

            // Apply power to the shooter motor based on the toggle state
            if (shooterOn) {
                shooterMotor.setPower(SHOOTER_POWER);
            } else {
                shooterMotor.setPower(0);
            }

            // --- D-Pad Control for Intake Motor ---
            if (gamepad2.dpad_left || gamepad2.dpad_right) {
                intakeMotor.setVelocity(0); // Stop intake
            } else if (gamepad2.dpad_up) {
                intakeMotor.setVelocity(INTAKE_VELOCITY); // Run intake in
            } else if (gamepad2.dpad_down) {
                intakeMotor.setVelocity(-INTAKE_VELOCITY); // Run intake out
            } else {
                intakeMotor.setVelocity(0); // Also stop if nothing is pressed
            }

            // --- Telemetry ---
            telemetry.addData("Shooter Status", shooterOn ? "ON" : "OFF");
            telemetry.addData("Shooter Power", shooterMotor.getPower());
            telemetry.addData("Intake Velocity", intakeMotor.getVelocity());
            telemetry.addLine("\nPress Right Trigger to toggle shooter.");
            telemetry.addLine("D-Pad Up/Down to run intake, Left/Right to stop.");
            telemetry.update();
        }
    }
}

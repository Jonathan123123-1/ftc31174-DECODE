package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Shooter and Stopper Test", group = "TEST")
public class ShooterStopperTest extends LinearOpMode {

    // --- Hardware ---
    private DcMotorEx shooterMotor;
    private Servo turretStopper;

    // --- Control Logic ---
    private boolean shooterOn = false;
    private boolean lastTriggerPressed = false;
    private boolean lastDpadRightPressed = false;

    private final double SHOOTER_POWER = 0.6;

    @Override
    public void runOpMode() {

        // --- Initialization ---
        try {
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
            turretStopper = hardwareMap.get(Servo.class, "turret_stopper");

            // It's good practice to set a direction and run mode for the motor
            shooterMotor.setDirection(DcMotorEx.Direction.FORWARD);
            shooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT); // Flywheels should float

            // Set stopper to home position
            turretStopper.setPosition(0);

        } catch (Exception e) {
            telemetry.addLine("ERROR: Could not find 'shooter_motor' or 'turret_stopper'.");
            telemetry.update();
            sleep(10000);
            return;
        }

        telemetry.addLine("Shooter & Stopper Test Ready.");
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

            // --- D-Pad Right: Cycle the Stopper Servo ---
            boolean dpadRightPressed = gamepad2.dpad_right;
            if (dpadRightPressed && !lastDpadRightPressed) {
                // 1. Move to push position
                turretStopper.setPosition(0.5);
                // 2. Wait a moment
                sleep(250);
                // 3. Return to home position
                turretStopper.setPosition(0);
            }
            lastDpadRightPressed = dpadRightPressed;

            // --- Telemetry ---
            telemetry.addData("Shooter Status", shooterOn ? "ON" : "OFF");
            telemetry.addData("Shooter Power", shooterMotor.getPower());
            telemetry.addData("Stopper Position", turretStopper.getPosition());
            telemetry.addLine("\nPress Right Trigger to toggle shooter.");
            telemetry.addLine("Press D-Pad Right to cycle stopper.");
            telemetry.update();
        }
    }
}

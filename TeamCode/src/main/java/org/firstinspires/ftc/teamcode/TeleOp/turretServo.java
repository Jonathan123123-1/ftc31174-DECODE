package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Turret Servo Test", group = "TeleOp")
public class turretServo extends LinearOpMode {

    // Declare the servo object
    private Servo turretServo;

    // This boolean is to detect a single button press
    private boolean lastDpadRightPressed = false;

    @Override
    public void runOpMode() {

        // Get the servo from the hardware map
        turretServo = hardwareMap.get(Servo.class, "turretStopper");

        // Reverse the servo's direction so that setting a position moves it the other way
        turretServo.setDirection(Servo.Direction.REVERSE);

        // Set the initial position of the servo to 0
        turretServo.setPosition(0.45);

        telemetry.addLine("Ready to test turret servo.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Get the current state of the d-pad button
            boolean dpadRightPressed = gamepad2.dpad_right;

            // Check if the button was just pressed (and not held down from the last loop)
            if (dpadRightPressed && !lastDpadRightPressed) {

                // --- The Sequence ---
                // 1. Move to 90 degrees (position 0.5) - this will now move "up"
                turretServo.setPosition(0.2);

                // 2. Wait for 500 milliseconds for the servo to move
                sleep(500);

                // 3. Return to the home position (0)
                turretServo.setPosition(0.45);
            }

            // Update the last known state of the button
            lastDpadRightPressed = dpadRightPressed;

            // Display the servo's current position on the driver station
            telemetry.addData("Servo Position", turretServo.getPosition());
            telemetry.update();
        }
    }
}

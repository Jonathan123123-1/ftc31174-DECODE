package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "LauncherHoldRB")
public class ServoSpinRB extends LinearOpMode {

    private Servo left_launcher;
    private Servo right_launcher;

    @Override
    public void runOpMode() {

        left_launcher  = hardwareMap.get(Servo.class, "left_launcher");
        right_launcher = hardwareMap.get(Servo.class, "right_launcher");

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.right_bumper) {
                // Hold to spin the launchers
                left_launcher.setPosition(0.0);      // full speed forward
                right_launcher.setPosition(0.0);     // full speed forward
            } else {
                // Release to stop
                left_launcher.setPosition(0.5);      // stop CR servo
                right_launcher.setPosition(0.5);     // stop CR servo
            }
        }
    }
}
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "LauncherForward5Sec")
public class LauncherForward5Sec extends LinearOpMode {

    private Servo left_launcher;
    private Servo right_launcher;

    @Override
    public void runOpMode() {

        left_launcher  = hardwareMap.get(Servo.class, "left_launcher");
        right_launcher = hardwareMap.get(Servo.class, "right_launcher");

        double posLeft = 0.5;
        double posRight = 0.5;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.right_bumper) {
                // Start timer
                long startTime = System.currentTimeMillis();

                while (System.currentTimeMillis() - startTime < 5000 && opModeIsActive()) {
                    // move fast forward
                    posLeft += 0.02;
                    posRight += 0.02;

                    // clamp positions
                    posLeft = Math.min(1, posLeft);
                    posRight = Math.min(1, posRight);

                    left_launcher.setPosition(posLeft);
                    right_launcher.setPosition(posRight);

                    sleep(10); // smooth movement
                }
            }
        }
    }
}

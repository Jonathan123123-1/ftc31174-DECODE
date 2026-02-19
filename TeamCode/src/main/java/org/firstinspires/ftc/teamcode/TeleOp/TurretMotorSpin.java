package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Turret Motor Spin")
public class TurretMotorSpin extends LinearOpMode {

    private DcMotor turretMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize turret motor
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");

        // Set motor direction if needed
        turretMotor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            // Check if right bumper is pressed
            if (gamepad1.right_bumper) {
                // Spin the motor at full power
                turretMotor.setPower(0.8);

                // Let it spin for 1 second (1000 ms)
                sleep(500);

                // Stop the motor
                turretMotor.setPower(0);
            } else {
                turretMotor.setPower(0);
            }

            idle(); // Let the system breathe
        }
    }
}
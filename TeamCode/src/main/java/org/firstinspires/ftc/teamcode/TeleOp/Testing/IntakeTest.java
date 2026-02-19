package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Intake Motor Test", group = "TEST")
public class IntakeTest extends LinearOpMode {

    private DcMotor intakeMotor;

    @Override
    public void runOpMode() {

        // --- Initialization ---
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            telemetry.addLine("ERROR: Could not find 'intake_motor'. Check hardware config.");
            telemetry.update();
            sleep(10000);
            return;
        }

        telemetry.addLine("Intake Test Ready.");
        telemetry.addLine("Use D-Pad on Gamepad 2 to control.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- Gamepad 2 Controls ---
            if (gamepad2.dpad_up) {
                // Run intake forward
                intakeMotor.setPower(1.0);
            } else if (gamepad2.dpad_down) {
                // Run intake in reverse
                intakeMotor.setPower(-1.0);
            } else if (gamepad2.dpad_left) {
                // Stop the intake
                intakeMotor.setPower(0.0);
            }

            // --- Telemetry ---
            telemetry.addData("Intake Power", intakeMotor.getPower());
            telemetry.addLine("D-Pad Up: Forward | D-Pad Down: Reverse | D-Pad Left: Stop");
            telemetry.update();
        }
    }
}

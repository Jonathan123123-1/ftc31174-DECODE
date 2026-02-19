package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Turret Slow Turn Test", group = "TEST")
public class TurretSlowTurnTest extends LinearOpMode {

    // --- Hardware ---
    private DcMotorEx turretMotor;

    // --- Turret State & Constants ---
    private enum TurretState { IDLE, MOVING_LEFT, MOVING_RIGHT }
    private TurretState currentState = TurretState.IDLE;

    // Based on REV Through Bore (8192) with a 5:1 ratio: (8192 * 5) / 360
    private final double TICKS_PER_DEGREE = 113.78;
    private final double SLOW_TURRET_POWER = 0.3; // Low power for slow, controlled movement

    // --- Gamepad Button Tracking ---
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    @Override
    public void runOpMode() {

        // --- Initialization ---
        try {
            turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
            // Set direction - if it spins the wrong way, change this to REVERSE
            turretMotor.setDirection(DcMotorEx.Direction.FORWARD);
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setTargetPosition(0);
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } catch (Exception e) {
            telemetry.addLine("ERROR: Could not find 'turret_motor'. Check hardware config.");
            telemetry.update();
            sleep(10000);
            return;
        }

        telemetry.addLine("Turret Slow Turn Test Ready.");
        telemetry.addLine("Use Left/Right Bumper on Gamepad 2.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- Gamepad Input (Gamepad 2) ---
            boolean leftBumperPressed = gamepad2.left_bumper;
            if (leftBumperPressed && !lastLeftBumper) {
                if (currentState == TurretState.MOVING_LEFT) {
                    // If we are already moving left, stop by going home
                    turretMotor.setTargetPosition(0);
                    currentState = TurretState.IDLE;
                } else {
                    // Start moving to the -90 degree position
                    int targetPosition = (int) (-90.0 * TICKS_PER_DEGREE);
                    turretMotor.setTargetPosition(targetPosition);
                    currentState = TurretState.MOVING_LEFT;
                }
            }
            lastLeftBumper = leftBumperPressed;

            boolean rightBumperPressed = gamepad2.right_bumper;
            if (rightBumperPressed && !lastRightBumper) {
                if (currentState == TurretState.MOVING_RIGHT) {
                    // If we are already moving right, stop by going home
                    turretMotor.setTargetPosition(0);
                    currentState = TurretState.IDLE;
                } else {
                    // Start moving to the +90 degree position
                    int targetPosition = (int) (90.0 * TICKS_PER_DEGREE);
                    turretMotor.setTargetPosition(targetPosition);
                    currentState = TurretState.MOVING_RIGHT;
                }
            }
            lastRightBumper = rightBumperPressed;

            // Set motor power only when it has a target to move to
            if (currentState != TurretState.IDLE) {
                turretMotor.setPower(SLOW_TURRET_POWER);
            } else {
                // If idle, hold position with a bit of power or relax it
                if (Math.abs(turretMotor.getCurrentPosition()) < 5) {
                    turretMotor.setPower(0); // Relax motor when near home
                } else {
                    turretMotor.setPower(SLOW_TURRET_POWER); // Hold position
                }
            }

            // --- Telemetry ---
            telemetry.addData("Turret State", currentState);
            telemetry.addData("Target Ticks", turretMotor.getTargetPosition());
            telemetry.addData("Current Ticks", turretMotor.getCurrentPosition());
            telemetry.addData("Current Degrees", turretMotor.getCurrentPosition() / TICKS_PER_DEGREE);
            telemetry.update();
        }
    }
}

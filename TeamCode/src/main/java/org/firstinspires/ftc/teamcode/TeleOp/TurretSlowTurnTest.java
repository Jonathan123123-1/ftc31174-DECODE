package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Turret Slow Turn Test - MANUAL PID", group = "TEST")
public class TurretSlowTurnTest extends LinearOpMode {

    // --- Hardware ---
    private DcMotor turretMotor;

    // --- Turret State & Constants ---
    private enum TurretState { IDLE, MOVING_TO_TARGET }
    private TurretState currentState = TurretState.IDLE;
    private int targetPosition = 0;

    // --- Manual PID Controller Constants ---
    // Tuned for a slower, more controlled motion with a hard lock.
    private double kP = 0.02;  // Proportional: Main driving force.
    private double kI = 0.007; // Integral: Overcomes friction to lock on target.
    private double kD = 0.03;  // Derivative: Prevents overshoot.
    private double integralSum = 0;
    private double lastError = 0;

    // --- Gamepad Button Tracking ---
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    @Override
    public void runOpMode() {

        // --- Initialization ---
        try {
            turretMotor = hardwareMap.get(DcMotor.class, "turret_motor");
            turretMotor.setDirection(DcMotor.Direction.REVERSE);
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            telemetry.addLine("FATAL ERROR: Could not find 'turret_motor'.");
            telemetry.update();
            sleep(10000);
            return;
        }

        telemetry.addLine("Turret Hard Stop PID Test Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- Gamepad Input (Gamepad 2) ---
            boolean leftBumperPressed = gamepad2.left_bumper;
            if (leftBumperPressed && !lastLeftBumper) {
                targetPosition = (targetPosition == -20) ? 0 : -20;
                integralSum = 0; // Reset PID for a clean move
                lastError = 0;
                currentState = TurretState.MOVING_TO_TARGET;
            }
            lastLeftBumper = leftBumperPressed;

            boolean rightBumperPressed = gamepad2.right_bumper;
            if (rightBumperPressed && !lastRightBumper) {
                targetPosition = (targetPosition == 20) ? 0 : 20;
                integralSum = 0; // Reset PID for a clean move
                lastError = 0;
                currentState = TurretState.MOVING_TO_TARGET;
            }
            lastRightBumper = rightBumperPressed;

            // --- State Machine & Manual PID Logic ---
            if (currentState == TurretState.MOVING_TO_TARGET) {
                int currentPosition = turretMotor.getCurrentPosition();
                int error = targetPosition - currentPosition;

                // Check if we have settled at the target
                if (Math.abs(error) < 3) { // 3-tick tolerance
                    currentState = TurretState.IDLE;
                } else {
                    integralSum += error;
                    double derivative = error - lastError;
                    lastError = error;
                    double power = (kP * error) + (kI * integralSum) + (kD * derivative);

                    // THE FIX: Decreased power cap for slower movement
                    power = Math.max(-0.3, Math.min(0.3, power));

                    turretMotor.setPower(power);
                }
            } 
            
            if (currentState == TurretState.IDLE) { 
                // Ensure motor power is zero to engage the BRAKE behavior
                turretMotor.setPower(0);
            }

            // --- Telemetry ---
            telemetry.addData("Turret State", currentState);
            telemetry.addData("Target Ticks", targetPosition);
            telemetry.addData("Current Ticks", turretMotor.getCurrentPosition());
            telemetry.addData("Motor Power", turretMotor.getPower());
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Shooter + Intake + Turret Stopper Test", group = "TEST")
public class ShooterIntakeTurretTest extends LinearOpMode {

    // --- Hardware ---
    private DcMotorEx shooterMotor;
    private DcMotorEx intakeMotor;
    private Servo turretServo;

    // --- Shooter Toggle ---
    private boolean shooterOn = false;
    private boolean lastTriggerPressed = false;

    // --- Turret Servo Toggle ---
    private boolean lastDpadRightPressed = false;

    // --- Constants ---
    private final double SHOOTER_POWER = 0.6;
    private final double INTAKE_VELOCITY = 1800;

    private final double TURRET_HOME = 0.45;
    private final double TURRET_ACTIVE = 0.2;

    @Override
    public void runOpMode() {

        try {
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
            turretServo = hardwareMap.get(Servo.class, "turretStopper");

            shooterMotor.setDirection(DcMotor.Direction.FORWARD);
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            intakeMotor.setDirection(DcMotor.Direction.FORWARD);
            intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            turretServo.setDirection(Servo.Direction.REVERSE);
            turretServo.setPosition(TURRET_HOME);

        } catch (Exception e) {
            telemetry.addLine("ERROR: Hardware not found.");
            telemetry.addLine(e.getMessage());
            telemetry.update();
            sleep(10000);
            return;
        }

        telemetry.addLine("Shooter, Intake, & Turret Stopper Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            /* ================= SHOOTER TOGGLE ================= */
            boolean triggerPressed = gamepad2.right_trigger > 0.5;
            if (triggerPressed && !lastTriggerPressed) {
                shooterOn = !shooterOn;
            }
            lastTriggerPressed = triggerPressed;

            shooterMotor.setPower(shooterOn ? SHOOTER_POWER : 0);

            /* ================= INTAKE CONTROL ================= */
            if (gamepad2.dpad_up) {
                intakeMotor.setVelocity(INTAKE_VELOCITY);
            } else if (gamepad2.dpad_down) {
                intakeMotor.setVelocity(-INTAKE_VELOCITY);
            } else {
                intakeMotor.setVelocity(0);
            }

            /* ================= TURRET STOPPER ================= */
            boolean dpadRightPressed = gamepad2.dpad_right;
            if (dpadRightPressed && !lastDpadRightPressed) {
                turretServo.setPosition(TURRET_ACTIVE);
                sleep(75);
                turretServo.setPosition(TURRET_HOME);
            }
            lastDpadRightPressed = dpadRightPressed;

            /* ================= TELEMETRY ================= */
            telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
            telemetry.addData("Shooter Power", shooterMotor.getPower());
            telemetry.addData("Intake Velocity", intakeMotor.getVelocity());
            telemetry.addData("Turret Servo Pos", turretServo.getPosition());
            telemetry.update();
        }
    }
}

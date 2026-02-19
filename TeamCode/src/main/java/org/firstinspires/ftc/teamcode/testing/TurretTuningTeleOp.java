package org.firstinspires.ftc.teamcode.testing;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooterConstants.TurretLockSystem;

/**
 * TurretTuningTeleOp
 * <p>
 * This TeleOp is designed for testing and tuning the TurretLockSystem.
 * It uses the specific hardware and localization setup defined in your
 * Constants.java file.
 * <p>
 * CONTROLS:
 * - Left Stick: Drive (forward/backward/strafe)
 * - Right Stick: Turn
 * - Gamepad 1 'A': Toggle turret lock on/off
 */
@TeleOp(name = "Turret Tuning TeleOp", group = "Testing")
public class TurretTuningTeleOp extends LinearOpMode {

    private Follower follower;
    private DcMotorEx turretMotor;
    private TurretLockSystem turretLockSystem;

    @Override
    public void runOpMode() throws InterruptedException {
        // ==================== INITIALIZATION ====================

        // Initialize the drive base and localizer using your Constants.java helper
        follower = Constants.createFollower(hardwareMap, telemetry);

        // Get the turret motor from the hardware map.
        // RENAME "turretMotor" if your configuration is different.
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");

        // Create the Turret Lock System, giving it the localizer from the follower
        turretLockSystem = new TurretLockSystem(
                follower.getLocalizer(), // Use the localizer from your follower
                turretMotor,
                telemetry
        );

        // You can set a custom target position here if needed.
        // The default is (15.0, 135.0) from TurretLockSystem.java
        // turretLockSystem.setTargetPosition(36, 72);

        // Gamepad effect for feedback
        Gamepad.RumbleEffect rumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.5, 0.5, 200)
                .build();

        telemetry.addLine("✅ Initialization Complete");
        telemetry.addLine("Press [A] on Gamepad 1 to toggle turret lock.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // ==================== OPMODE LOOP ====================

        while (opModeIsActive()) {

            // --- Drive Control ---
            // This uses your existing follower and drive constants
            follower.setDrivePowers(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            );

            // --- Turret Lock Control ---
            // Simple toggle logic with debounce to prevent rapid switching
            if (gamepad1.a && !gamepad1.start) {
                turretLockSystem.toggleLock();
                gamepad1.runRumbleEffect(rumbleEffect);
            }
            // Debounce by using a button that's unlikely to be pressed (start)
            gamepad1.start = gamepad1.a;


            // --- Core System Updates ---
            // These MUST be called in every loop iteration
            follower.update();
            turretLockSystem.update();

            // Telemetry is handled by the turretLockSystem itself
            telemetry.update();
        }
    }
}

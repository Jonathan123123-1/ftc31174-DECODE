package org.firstinspires.ftc.teamcode.testing;

import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooterConstants.LimelightVision;
import org.firstinspires.ftc.teamcode.shooterConstants.TurretLockSystem;
import org.firstinspires.ftc.teamcode.shooterConstants.fusedLocalizer;

/**
 * TurretTuningTeleOp (Version 3 - Cleaned)
 * <p>
 * This TeleOp is for testing the TurretLockSystem.
 * <p>
 * CONTROLS:
 * - Gamepad 1 Left Stick: Drive (forward/backward/strafe)
 * - Gamepad 1 Right Stick: Turn
 * - Gamepad 1 'A' (HOLD): Lock the turret onto the target.
 * - Gamepad 1 'A' (RELEASE): Return turret to zero.
 */
@TeleOp(name = "Turret Tuning TeleOp", group = "Testing")
public class TurretTuningTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // ==================== 1. INITIALIZE LOCALIZER ====================
        // This is your custom localizer that fuses odometry and vision.
        // It's the brain that tells the robot where it is.
        fusedLocalizer robotLocalizer = new fusedLocalizer(
                new PinpointLocalizer(hardwareMap, Constants.localizerConstants),
                new LimelightVision(hardwareMap),
                telemetry
        );

        // ==================== 2. INITIALIZE DRIVETRAIN ====================
        // The Follower handles the robot's movement. We give it the localizer
        // so it knows where it is while driving.
        Follower follower = new FollowerBuilder(Constants.followerConstants, hardwareMap)
                .setLocalizer(robotLocalizer)
                .pathConstraints(Constants.pathConstraints)
                .mecanumDrivetrain(Constants.driveConstants)
                .build();

        // ==================== 3. INITIALIZE TURRET HARDWARE ====================
        DcMotorEx turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD); // IMPORTANT: Change to REVERSE if it turns the wrong way.

        // ==================== 4. INITIALIZE TURRET LOCK SYSTEM ====================
        // This is our new system. We give it the same localizer so it can calculate
        // the angle to the target based on the robot's position.
        TurretLockSystem turretLockSystem = new TurretLockSystem(
                robotLocalizer, // Use the same localizer as the drivetrain
                turretMotor,
                telemetry
        );

        telemetry.addLine("✅ Initialization Complete");
        telemetry.addLine("Press and HOLD [A] to lock turret.");
        telemetry.update();

        waitForStart();

        // ==================== OPMODE LOOP ====================
        while (opModeIsActive()) {

            // --- Turret Lock Control ---
            // If the 'A' button is held down, tell the system to lock on.
            // If it's released, tell the system to unlock.
            if (gamepad1.a) {
                turretLockSystem.lockOn();
            } else {
                turretLockSystem.unlock();
            }

            // --- System Updates ---
            // These methods must be called in every single loop.
            // They read sensor data and apply power to the motors.
            follower.update();
            turretLockSystem.update();

            // Display telemetry from the turret system.
            telemetry.update();
        }
    }
}

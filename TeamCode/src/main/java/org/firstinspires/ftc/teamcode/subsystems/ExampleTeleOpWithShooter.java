package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.*;

// Uncomment these when you're ready to integrate Pedro Pathing:
// import com.pedropathing.follower.Follower;
// import com.pedropathing.localization.Pose;

/**
 * TeleOp showing shooter integration with Pedro Pathing
 * This example shows how to integrate the shooter with your existing drive code
 */
@TeleOp(name="Shooter TeleOp (Pedro)", group="Competition")
public class ExampleTeleOpWithShooter extends LinearOpMode {

    private ShooterSubsystem shooter;
    // Uncomment when you're ready to integrate:
    // private Follower follower;

    // Alliance selection - SET THIS BEFORE MATCH!
    private boolean isRedAlliance = true;

    @Override
    public void runOpMode() {
        // Initialize shooter subsystem
        shooter = new ShooterSubsystem(hardwareMap);

        // TODO: Initialize your Pedro Pathing follower here
        // follower = new Follower(hardwareMap);
        // follower.setStartingPose(new Pose(0, 0, 0));

        telemetry.addLine("Shooter TeleOp Ready");
        telemetry.addLine("=====================================");
        telemetry.addLine("Right Bumper: Aim at goal");
        telemetry.addLine("Right Trigger: Shoot when ready");
        telemetry.addLine("Left Bumper: Toggle alliance color");
        telemetry.addLine("=====================================");
        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // =================================================================
            // DRIVE CONTROL (Your existing code goes here)
            // =================================================================
            // TODO: Add your mecanum drive code or Pedro Pathing control

            // =================================================================
            // GET ROBOT POSITION AND VELOCITY FROM PEDRO PATHING
            // =================================================================

            // Get current pose from Pedro Pathing
            // TODO: Uncomment and use your actual follower
            /*
            Pose currentPose = follower.getPose();
            double robotX = currentPose.getX();      // inches (0-144)
            double robotY = currentPose.getY();      // inches (0-144)
            double robotHeading = currentPose.getHeading(); // radians

            // Get velocity from Pedro Pathing
            com.pedropathing.util.Vector velocity = follower.getVelocity();
            double robotVx = velocity.getXComponent();  // inches per second
            double robotVy = velocity.getYComponent();  // inches per second
            */

            // TEMPORARY: For testing without odometry, use fixed values
            double robotX = 72.0;      // Center of field
            double robotY = 72.0;      // Center of field
            double robotHeading = 0.0; // Facing right
            double robotVx = 0.0;      // Not moving
            double robotVy = 0.0;      // Not moving

            // =================================================================
            // ALLIANCE TOGGLE
            // =================================================================

            if (gamepad1.left_bumper) {
                isRedAlliance = !isRedAlliance;
                sleep(200); // Debounce
            }

            // =================================================================
            // SHOOTER CONTROL
            // =================================================================

            // Right bumper: Calculate shot for current position and velocity
            if (gamepad1.right_bumper) {
                shooter.calculateShot(
                        robotX, robotY, robotHeading,
                        robotVx, robotVy,
                        isRedAlliance
                );
            }

            // Right trigger: Shoot when ready
            if (gamepad1.right_trigger > 0.5) {
                if (shooter.isReady()) {
                    // TODO: Activate your ball feeding mechanism here
                    // feedBall();
                    telemetry.addLine(">>> SHOOTING! <<<");
                } else {
                    telemetry.addLine(">>> NOT READY - WAIT <<<");
                }
            }

            // Update shooter motors (CRITICAL - must be called every loop!)
            shooter.update();

            // =================================================================
            // TELEMETRY
            // =================================================================

            telemetry.addLine("=== ROBOT STATE ===");
            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            telemetry.addData("Position", "(%.1f, %.1f)", robotX, robotY);
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(robotHeading));
            telemetry.addData("Velocity", "%.1f in/s",
                    Math.sqrt(robotVx * robotVx + robotVy * robotVy));
            telemetry.addLine();

            // Shooter telemetry
            telemetry.addLine(shooter.getTelemetry());
            telemetry.addLine();

            // Visual ready indicator
            if (shooter.isReady()) {
                telemetry.addLine("╔═══════════════════════════════╗");
                telemetry.addLine("║   >>> READY TO SHOOT <<<     ║");
                telemetry.addLine("║   Hold Right Trigger to Fire ║");
                telemetry.addLine("╚═══════════════════════════════╝");
            } else {
                telemetry.addLine("Waiting for motors to reach targets...");
                telemetry.addLine("Hold Right Bumper to aim");
            }

            telemetry.update();
        }

        // Stop shooter when OpMode ends
        shooter.stop();
    }
}
package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "AutoBlue1", group = "Autonomous")
public class AutoBlue1 extends LinearOpMode {

    @Override
    public void runOpMode() {

        // --- Initialize Pedro Pathing ---
        Follower follower = Constants.createFollower(hardwareMap);

        // --- Define the Autonomous Path ---
        // A PathChain is built by adding sequential path segments.
        // We'll start at our current pose, which is (0,0) by default.
        PathChain pathToGoal = follower.pathBuilder()
                // Move 48 inches backward (negative X)
                .addPath(new BezierLine(follower.getPose(), new Pose(48, 9.2, 0)))
                // From there, strafe 24 inches left (positive Y)
                .addPath(new BezierLine(new Pose(48, 0, 0), new Pose(48, 24, 0)))
                .build();

        telemetry.addLine("Initialized and ready to start!");
        telemetry.addLine("Path: Move back 48 inches, then strafe left 24 inches.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Set the path for the follower to drive.
            follower.followPath(pathToGoal);

            // This loop will run until the path is complete.
            // follower.isBusy() returns true while the path is still running.
            while(opModeIsActive() && follower.isBusy()) {
                // This line is critical. It sends power to the motors and updates the robot's position.
                follower.update();
                // You can add telemetry here to monitor the robot's position in real-time
                telemetry.addData("X", follower.getPose().getX());
                telemetry.addData("Y", follower.getPose().getY());
                telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
                telemetry.update();
            }
        }

        telemetry.addLine("Autonomous routine complete");
        telemetry.update();
        sleep(1000);
    }
}

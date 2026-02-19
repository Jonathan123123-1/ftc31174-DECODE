package org.firstinspires.ftc.teamcode.Meet3;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Manual Shooter TeleOp - RED", group = "COMPETITION")
public class ManuelShooterTeleOpBlue extends OpMode {

    // --- Hardware ---
    private DcMotorEx turretMotor, shooterMotor, intakeMotor;
    private Servo turretStopper;
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private IMU imu;
    private Follower follower;

    // --- Control Constants ---
    private final double TURRET_MANUAL_POWER = 0.5;
    private final double INTAKE_VELOCITY = 4000;
    private final double SHOOTER_HIGH_RPM = 1545; // B button locks to this RPM
    private final double SHOOTER_LOW_RPM = 1280;  // A button locks to this RPM
    private final double STOPPER_ENGAGED_POSITION = 0;
    private final double STOPPER_RELEASED_POSITION = 0.5;

    // --- Shooter PIDF ---
    private final double SHOOTER_P = 30.0;
    private final double SHOOTER_I = 5.0;
    private final double SHOOTER_D = 0.0;
    private final double SHOOTER_F = 25.0;

    // --- Logic Variables ---
    private boolean fieldOriented = false;
    private boolean lastToggleX = false;
    private boolean slowMode = false;
    private boolean lastGamepad1A = false;
    private boolean lastGamepad2A = false;
    private boolean lastGamepad2B = false;
    private boolean lastGamepad2Y = false;
    private boolean lastGamepad2X = false;
    private boolean stopperEngaged = false;
    private double currentTargetRPM = 0;


    @Override
    public void init() {
        // Drivetrain & IMU
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        imu = hardwareMap.get(IMU.class, "imu");

        // Scoring Mechanisms
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        turretStopper = hardwareMap.get(Servo.class, "turretStopper");

        // Set motor modes
        turretMotor.setDirection(DcMotorEx.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Manual control
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotor.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretStopper.setPosition(STOPPER_RELEASED_POSITION);

        // Initialize follower for position tracking
        PinpointConstants correctedLocalizer = new PinpointConstants()
                .forwardPodY(183.16/2.54)
                .strafePodX(-63.5/2.54)
                .distanceUnit(DistanceUnit.MM)
                .hardwareMapName("pinpoint")
                .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
                .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
                .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

        follower = new FollowerBuilder(Constants.followerConstants, hardwareMap)
                .pinpointLocalizer(correctedLocalizer)
                .pathConstraints(Constants.pathConstraints)
                .mecanumDrivetrain(Constants.driveConstants)
                .build();
        follower.setStartingPose(new Pose(0,5,0)); // Set a default starting pose

        telemetry.addLine("Manual Shooter TeleOp Initialized.");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        handleDriving();
        handleManualShooterControls();
        updateTelemetry();
    }

    private void handleManualShooterControls() {
        // --- Intake Control ---
        if (gamepad2.dpad_up) {
            intakeMotor.setVelocity(-INTAKE_VELOCITY);
        } else if (gamepad2.dpad_down) {
            intakeMotor.setVelocity(INTAKE_VELOCITY);
        } else {
            intakeMotor.setVelocity(0);
        }

        // --- Turret Stopper ---
        if (gamepad2.x && !lastGamepad2X) {
            stopperEngaged = !stopperEngaged;
            turretStopper.setPosition(stopperEngaged ? STOPPER_ENGAGED_POSITION : STOPPER_RELEASED_POSITION);
        }
        lastGamepad2X = gamepad2.x;

        // --- RPM Lock Selection ---
        if (gamepad2.b && !lastGamepad2B) { // B button locks to High RPM
            currentTargetRPM = SHOOTER_HIGH_RPM;
        }
        if (gamepad2.a && !lastGamepad2A) { // A button locks to Low RPM
            currentTargetRPM = SHOOTER_LOW_RPM;
        }
        if (gamepad2.y && !lastGamepad2Y) { // Y button turns shooter off
            currentTargetRPM = 0;
        }
        lastGamepad2A = gamepad2.a;
        lastGamepad2B = gamepad2.b;
        lastGamepad2Y = gamepad2.y;

        // Set the shooter motor velocity to the currently locked RPM
        shooterMotor.setVelocity(currentTargetRPM);

        // --- Manual Turret Control ---
        turretMotor.setPower(-gamepad2.left_stick_y * TURRET_MANUAL_POWER);
    }

    private void handleDriving() {
        if (gamepad1.x && !lastToggleX) fieldOriented = !fieldOriented;
        lastToggleX = gamepad1.x;

        if (gamepad1.a && !lastGamepad1A) slowMode = !slowMode;
        lastGamepad1A = gamepad1.a;

        double maxSpeed = slowMode ? 0.4 : 1.0;
        double y = -gamepad1.left_stick_y * maxSpeed;
        double x = gamepad1.left_stick_x * maxSpeed;
        double rx = gamepad1.right_stick_x * maxSpeed;

        if (fieldOriented) {
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw();
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            x = rotX; y = rotY;
        }

        double den = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftDrive.setPower((y + x + rx) / den);
        backLeftDrive.setPower((y - x + rx) / den);
        frontRightDrive.setPower((y - x - rx) / den);
        backRightDrive.setPower((y + x - rx) / den);
    }

    private void updateTelemetry() {
        Pose robotPose = follower.getPose();
        telemetry.addLine("--- MANUAL SHOOTER MODE ---");
        telemetry.addData("Drive Mode", (fieldOriented ? "FIELD" : "ROBOT") + (slowMode ? " | SLOW" : ""));
        telemetry.addData("Target RPM", "%.1f", currentTargetRPM);
        telemetry.addData("Turret Power", turretMotor.getPower());
        telemetry.addData("Intake Velocity", intakeMotor.getVelocity());
        telemetry.addLine("\n--- ROBOT POSE ---");
        telemetry.addData("X", "%.2f", robotPose.getX());
        telemetry.addData("Y", "%.2f", robotPose.getY());
        telemetry.addData("Heading", "%.2f", Math.toDegrees(robotPose.getHeading()));
        telemetry.update();
    }
}

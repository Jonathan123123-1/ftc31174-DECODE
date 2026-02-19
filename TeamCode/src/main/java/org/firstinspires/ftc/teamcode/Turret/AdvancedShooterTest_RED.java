package org.firstinspires.ftc.teamcode.Turret;

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

@TeleOp(name = "Advanced Shooter Test - RED", group = "TEST")
public class AdvancedShooterTest_RED extends OpMode {

    // --- Hardware ---
    private DcMotorEx turretMotor, shooterMotor;
    private Servo hoodServo;
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private IMU imu;
    private Follower follower;

    // --- State Machine ---
    private enum SystemState { SYSTEM_OFF, SYSTEM_ACTIVE, HOMING }
    private SystemState systemState = SystemState.SYSTEM_OFF;

    // --- Control Constants ---
    private final double TURRET_TICKS_PER_DEGREE = 1.4936;
    private final double TURRET_POWER = 0.8;
    private final int TURRET_HOME_POSITION = 0;
    private final int MAX_TURRET_TICKS = 270;
    private final int MIN_TURRET_TICKS = -270;

    // --- Logic Variables ---
    private boolean lastRightBumper = false;
    private boolean fieldOriented = false;
    private boolean lastToggleX = false;
    private boolean slowMode = false;
    private boolean lastToggleLeftBumper = false;

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

        PinpointConstants correctedLocalizer = new PinpointConstants()
                .forwardPodY(183.16/2.54).strafePodX(-63.5/2.54)
                .distanceUnit(DistanceUnit.MM).hardwareMapName("pinpoint")
                .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
                .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
                .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

        follower = new FollowerBuilder(Constants.followerConstants, hardwareMap)
                .pinpointLocalizer(correctedLocalizer).pathConstraints(Constants.pathConstraints).mecanumDrivetrain(Constants.driveConstants).build();

        // Scoring Mechanisms
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        hoodServo = hardwareMap.get(Servo.class, "turret_servo");
        turretMotor.setDirection(DcMotorEx.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(TURRET_HOME_POSITION);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        
        telemetry.addLine("Advanced Shooter Test Initialized.");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        handleDriving();
        handleSystemControls();
        updateTelemetry();
    }

    private void handleSystemControls() {
        boolean rightBumper = gamepad1.right_bumper;
        if (rightBumper && !lastRightBumper) {
            systemState = (systemState == SystemState.SYSTEM_ACTIVE) ? SystemState.HOMING : SystemState.SYSTEM_ACTIVE;
        }
        lastRightBumper = rightBumper;

        switch(systemState) {
            case SYSTEM_ACTIVE:
                // On every loop, calculate the perfect shot and command the hardware
                localizationTesting.LaunchVector solution = localizationTesting.calculateLaunchVector(follower.getPose(), follower.getVelocity(), true);
                if (solution != null) {
                    // Convert theoretical values to hardware commands
                    double flywheelTicks = localizationTesting.getFlywheelTicksFromVelocity(solution.flywheelVelocity);
                    double hoodServoPos = localizationTesting.getHoodTicksFromDegrees(solution.hoodAngle);

                    // Command hardware
                    shooterMotor.setVelocity(flywheelTicks);
                    hoodServo.setPosition(hoodServoPos);
                    
                    int targetPosition = (int) (solution.turretAngle * TURRET_TICKS_PER_DEGREE);
                    int clampedPosition = Math.max(MIN_TURRET_TICKS, Math.min(targetPosition, MAX_TURRET_TICKS));
                    turretMotor.setTargetPosition(clampedPosition);
                    turretMotor.setPower(TURRET_POWER);
                } else {
                    // If no solution, turn off shooter
                    shooterMotor.setVelocity(0);
                }
                break;
            case HOMING:
                turretMotor.setTargetPosition(TURRET_HOME_POSITION);
                shooterMotor.setPower(0);
                if (Math.abs(turretMotor.getCurrentPosition() - TURRET_HOME_POSITION) < 15) {
                    systemState = SystemState.SYSTEM_OFF;
                }
                break;
            case SYSTEM_OFF:
                turretMotor.setPower(0);
                shooterMotor.setPower(0);
                break;
        }
    }
    
    private void handleDriving() {
        if (gamepad1.x && !lastToggleX) fieldOriented = !fieldOriented;
        lastToggleX = gamepad1.x;
        if (gamepad1.left_bumper && !lastToggleLeftBumper) slowMode = !slowMode;
        lastToggleLeftBumper = gamepad1.left_bumper;

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
        frontLeftDrive.setPower((y + x + rx) / den); backLeftDrive.setPower((y - x + rx) / den);
        frontRightDrive.setPower((y - x - rx) / den); backRightDrive.setPower((y + x - rx) / den);
    }

    private void updateTelemetry() {
        Pose robotPose = follower.getPose();
        telemetry.addData("System State", systemState);
        telemetry.addData("Drive Mode", (fieldOriented ? "FIELD" : "ROBOT") + (slowMode ? " | SLOW" : ""));
        telemetry.addData("Turret Target", turretMotor.getTargetPosition());
        telemetry.addData("Shooter Velocity", shooterMotor.getVelocity());
        telemetry.addData("Robot X", "%.2f", robotPose.getX());
        telemetry.addData("Robot Y", "%.2f", robotPose.getY());
        telemetry.addData("Robot Heading", "%.2f", Math.toDegrees(robotPose.getHeading()));
        telemetry.update();
    }
}

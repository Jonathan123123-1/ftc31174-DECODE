package org.firstinspires.ftc.teamcode.Turret;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.pedroPathing.teleConstants;
import org.firstinspires.ftc.teamcode.shooterConstants.distanceLocalization;

@TeleOp(name = "Odometry Turret Lock (Red)", group = "Turret")
public class odometryTurretLocalizationRed extends OpMode {

    // --- Hardware & State ---
    private DcMotorEx turretMotor;
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private IMU imu;
    private Follower follower; // Pedro Pathing for odometry

    private enum TurretState { IDLE, TRACKING, HOMING }
    private TurretState currentState = TurretState.IDLE;

    // --- Control Constants ---
    private final double TURRET_TICKS_PER_DEGREE = 1.4936; 
    private final double TURRET_POWER = 0.8;
    private final int TURRET_HOME_POSITION = 0;
    private final int MAX_TURRET_TICKS = 20;
    private final int MIN_TURRET_TICKS = -20;

    // --- Logic Variables ---
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;
    private boolean fieldOriented = false;
    private boolean lastToggleX = false;

    @Override
    public void init() {
        // Drivetrain
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        imu = hardwareMap.get(IMU.class, "imu");

        // Odometry
        follower = teleConstants.createFollower(hardwareMap,telemetry);
        follower.setStartingPose(new Pose(0,0,0));

        // Turret
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        turretMotor.setDirection(DcMotorEx.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(TURRET_HOME_POSITION);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(TURRET_POWER);

        telemetry.addLine("Odometry Turret Lock Initialized - RED Alliance");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        handleDriving();
        handleTurretLogic();
        updateTelemetry();
    }

    private void handleTurretLogic() {
        // --- Bumper Input ---
        boolean rightBumper = gamepad2.right_bumper;
        if (rightBumper && !lastRightBumper) {
            currentState = (currentState == TurretState.TRACKING) ? TurretState.HOMING : TurretState.TRACKING;
        }
        lastRightBumper = rightBumper;

        boolean leftBumper = gamepad2.left_bumper;
        if (leftBumper && !lastLeftBumper) {
            currentState = (currentState == TurretState.TRACKING) ? TurretState.HOMING : TurretState.TRACKING;
        }
        lastLeftBumper = leftBumper;

        // --- State Machine ---
        switch (currentState) {
            case TRACKING:
                Pose robotPose = follower.getPose();
                // Use TRUE for Red Alliance
                double targetAngleRad = distanceLocalization.getTargetTurretAngle(robotPose, true);
                double targetAngleDeg = Math.toDegrees(targetAngleRad);

                int targetPosition = (int) (targetAngleDeg * TURRET_TICKS_PER_DEGREE);

                int clampedPosition = Math.max(MIN_TURRET_TICKS, Math.min(targetPosition, MAX_TURRET_TICKS));
                
                turretMotor.setTargetPosition(clampedPosition);
                turretMotor.setPower(TURRET_POWER);
                break;

            case HOMING:
                turretMotor.setTargetPosition(TURRET_HOME_POSITION);
                turretMotor.setPower(TURRET_POWER);
                if (Math.abs(turretMotor.getCurrentPosition() - TURRET_HOME_POSITION) < 15) {
                    currentState = TurretState.IDLE;
                }
                break;

            case IDLE:
                turretMotor.setPower(0); // Relax motor
                break;
        }
    }

    private void handleDriving() {
        if (gamepad1.x && !lastToggleX) {
            fieldOriented = !fieldOriented;
        }
        lastToggleX = gamepad1.x;

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (fieldOriented) {
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw();
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            x = rotX;
            y = rotY;
        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftDrive.setPower((y + x + rx) / denominator);
        backLeftDrive.setPower((y - x + rx) / denominator);
        frontRightDrive.setPower((y - x - rx) / denominator);
        backRightDrive.setPower((y + x - rx) / denominator);
    }

    private void updateTelemetry() {
        Pose robotPose = follower.getPose();
        telemetry.addData("Turret State", currentState);
        telemetry.addData("Turret Target Ticks", turretMotor.getTargetPosition());
        telemetry.addData("Robot X", "%.2f", robotPose.getX());
        telemetry.addData("Robot Y", "%.2f", robotPose.getY());
        telemetry.addData("Robot Heading", "%.2f", Math.toDegrees(robotPose.getHeading()));
        telemetry.addData("Field Oriented", fieldOriented);
        telemetry.update();
    }
}

package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.shooterConstants.ShooterConstants;

@TeleOp(name = "DUAL Controls - Test Blue", group = "TEST")
public class Dual_TestBlue extends LinearOpMode {

    /* ================= DRIVE ================= */
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private IMU imu;

    private double maxSpeed = 0.9;
    private boolean fieldOriented = false;
    private boolean lastFieldToggle = false;

    /* ================= MECHANISMS ================= */
    private DcMotorEx shooterMotor;
    private DcMotorEx intakeMotor;
    private Servo turretServo;
    private Servo hoodServo;

    private DcMotorEx turretMotor;

    private Limelight3A limelight;

    private boolean shooterOn = false;
    private boolean lastTriggerPressed = false;

    /* ================= TURRET STATE ================= */
    private boolean turretActive = false;
    private long turretStartTime = 0;

    /* ================= SHOOTER CONSTANTS ================= */
    private static final double FAR_VELOCITY = 1545;   // 1570 RPM
    private static final double CLOSE_VELOCITY = 1260; // 1250 RPM

    private static final double SHOOTER_kP = 185.0;
    private static final double SHOOTER_kI = 13.0;
    private static final double SHOOTER_kD = 0.0;
    private static final double SHOOTER_kF = 18;

    private double shooterTargetVelocity = CLOSE_VELOCITY;

    /* ================= OTHER CONSTANTS ================= */
    private static final double INTAKE_VELOCITY = 1800;

    private static final double TURRET_HOME = 0.45;
    private static final double TURRET_ACTIVE = 0.2;
    private static final long TURRET_TIME_MS = 1500;
    private static final double LimeLight_kP = 0.012;//0.012

    /* ================= LIMELIGHT CONSTANTS ================= */
    // TODO: Tune these values for your specific robot and target
    private static final double CAMERA_HEIGHT_INCHES = 12.0; // Example: height of the camera lens from the floor
    private static final double TARGET_HEIGHT_INCHES = 29.5; // Example: height of the AprilTag center from the floor for CENTERSTAGE
    private static final double CAMERA_MOUNT_ANGLE_DEGREES = 0.0; // Example: camera mounting angle on the robot


    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(8);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();
        /* ---------- Hardware Init ---------- */
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        turretServo = hardwareMap.get(Servo.class, "turretStopper");
        hoodServo = hardwareMap.get(Servo.class, "turret_servo");


        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setVelocityPIDFCoefficients(
                SHOOTER_kP,
                SHOOTER_kI,
                SHOOTER_kD,
                SHOOTER_kF
        );

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretServo.setDirection(Servo.Direction.REVERSE);
        turretServo.setPosition(TURRET_HOME);

        telemetry.addLine("ALL SYSTEMS READY — SHOOTER PIDF ACTIVE");
        telemetry.update();

        waitForStart();

        /* ================= MAIN LOOP ================= */
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            double distance = 180;
            if (result.isValid()) {
                double ty = result.getTy();
                // This formula uses trigonometry to calculate the distance to the target.
                // It's based on the known heights of the AprilTag and the camera,
                // as well as the camera's mounting angle.
                distance = (TARGET_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) /
                        Math.tan(Math.toRadians(CAMERA_MOUNT_ANGLE_DEGREES + ty));
                telemetry.addData("Distance (in)", "%.2f", distance);
            } else {
                telemetry.addData("Distance (in)", "No target visible");
            }

            /* ---------- DRIVE ---------- */
            boolean fieldToggle = gamepad1.x;
            if (fieldToggle && !lastFieldToggle) {
                fieldOriented = !fieldOriented;
            }
            lastFieldToggle = fieldToggle;

            if (gamepad1.a) maxSpeed = 0.6;
            if (gamepad1.b) maxSpeed = 0.88;

            double forward = -gamepad1.left_stick_y;
            double right = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            if (fieldOriented) {
                double heading = imu.getRobotYawPitchRollAngles().getYaw();
                double temp = forward * Math.cos(heading) + right * Math.sin(heading);
                right = -forward * Math.sin(heading) + right * Math.cos(heading);
                forward = temp;
            }

            drive(forward, right, rotate);

            /* ---------- SHOOTER ZONE SELECT ---------- */
            if (gamepad2.x) shooterTargetVelocity = FAR_VELOCITY;
            if (gamepad2.y) shooterTargetVelocity = CLOSE_VELOCITY;

            /* ---------- SHOOTER TOGGLE ---------- */
            boolean triggerPressed = gamepad2.right_trigger > 0.5;
            if (triggerPressed && !lastTriggerPressed) {
                shooterOn = !shooterOn;
            }
            lastTriggerPressed = triggerPressed;

            if (shooterOn) {
                shooterMotor.setVelocity(shooterTargetVelocity);
            } else {
                shooterMotor.setVelocity(0);
            }

            /* ---------- INTAKE ---------- */
            if (gamepad1.right_bumper) {
                intakeMotor.setVelocity(-INTAKE_VELOCITY);
            } else if (gamepad1.right_trigger > 0.2) {
                intakeMotor.setVelocity(INTAKE_VELOCITY);
            } else {
                intakeMotor.setVelocity(0);
            }

            /*--------When only 1 Ball--------*/
            if (gamepad2.right_trigger >0.8) {
                intakeMotor.setVelocity(INTAKE_VELOCITY);
            }

            /* ---------- TURRET SERVO ---------- */
            if (gamepad2.right_bumper && !turretActive) {
                turretServo.setPosition(TURRET_ACTIVE);
                turretStartTime = System.currentTimeMillis();
                turretActive = true;
            }

            if (turretActive &&
                    System.currentTimeMillis() - turretStartTime >= TURRET_TIME_MS) {
                turretServo.setPosition(TURRET_HOME);
                turretActive = false;
            }

            if(gamepad2.left_bumper){
                hoodServo.setPosition(ShooterConstants.hoodPosition(distance));
                shooterTargetVelocity=ShooterConstants.targetRPM(distance);
            }
            if(gamepad2.left_bumper && result.isValid()&&Math.abs(result.getTx())>3){
                turretMotor.setPower(-result.getTx()*LimeLight_kP+Math.signum(-result.getTx())*.085);
            }else{
                turretMotor.setPower(0);
            }

            /* ---------- TELEMETRY ---------- */
            telemetry.addData("Shooter Target", shooterTargetVelocity);
            telemetry.addData("Shooter Actual", shooterMotor.getVelocity());
            telemetry.addData("Intake Vel", intakeMotor.getVelocity());
            telemetry.addData("Turret Pos", turretServo.getPosition());
            telemetry.addData("Tx: ", result.getTx());


            telemetry.update();
        }
    }

    private void drive(double forward, double right, double rotate) {
        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double bl = forward - right + rotate;
        double br = forward + right - rotate;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        frontLeftDrive.setPower(fl * maxSpeed);
        frontRightDrive.setPower(fr * maxSpeed);
        backLeftDrive.setPower(bl * maxSpeed);
        backRightDrive.setPower(br * maxSpeed);
    }
}

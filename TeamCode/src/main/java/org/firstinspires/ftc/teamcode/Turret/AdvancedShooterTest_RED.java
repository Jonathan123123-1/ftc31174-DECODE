package org.firstinspires.ftc.teamcode.Turret;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Decode FTC Implementation", group = "TEST")
public class AdvancedShooterTest_RED extends OpMode {

    private DcMotorEx turretMotor, shooterMotor;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Servo hoodServo;
    private Follower follower;

    // High Reactivity PIDF for Compression
    private final double SHOOTER_P = 35.0;
    private final double SHOOTER_F = 15.0;
    private final double TURRET_P = 0.025;

    private boolean active = false;
    private boolean lastRB = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap,telemetry);

        // Drive Motors
        frontLeft = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRight = hardwareMap.get(DcMotor.class, "back_right_drive");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Hardware
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        hoodServo = hardwareMap.get(Servo.class, "turret_servo");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setVelocityPIDFCoefficients(SHOOTER_P, 0, 0, SHOOTER_F);
    }

    @Override
    public void start() { follower.startTeleopDrive(); }

    @Override
    public void loop() {
        follower.update();
        handleManualDrive();

        if (gamepad1.right_bumper && !lastRB) active = !active;
        lastRB = gamepad1.right_bumper;

        if (active) {
            localizationTesting.LaunchVector sol = localizationTesting.calculateLaunchVector(
                    follower.getPose(), follower.getVelocity(), true);

            if (sol != null) {
                // Turret Aim
                double error = sol.turretAngle - (turretMotor.getCurrentPosition() / 1.4936);
                turretMotor.setPower(Range.clip(error * TURRET_P, -0.4, 0.4));

                // Flywheel & Hood
                shooterMotor.setVelocity(localizationTesting.getFlywheelTicks(sol.flywheelVelocity));
                hoodServo.setPosition(localizationTesting.getHoodServoPos(sol.hoodAngle));
            }
        } else {
            turretMotor.setPower(0);
            shooterMotor.setPower(0);
        }
    }

    private void handleManualDrive() {
        double y = -gamepad1.left_stick_y, x = gamepad1.left_stick_x, rx = gamepad1.right_stick_x;
        double den = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeft.setPower((y + x + rx) / den);
        backLeft.setPower((y - x + rx) / den);
        frontRight.setPower((y - x - rx) / den);
        backRight.setPower((y + x - rx) / den);
    }
}
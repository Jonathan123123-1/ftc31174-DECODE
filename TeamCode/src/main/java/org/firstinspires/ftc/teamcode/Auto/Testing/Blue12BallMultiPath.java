package org.firstinspires.ftc.teamcode.Auto.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Blue 12 Ball Multi-Path", group = "Autonomous")
@Configurable
public class Blue12BallMultiPath extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private ElapsedTime StateTimer = new ElapsedTime();
    private ElapsedTime advanceTimer = new ElapsedTime();
    private boolean isWaiting = false;
    private int pathState;
    private Paths paths;
    private int lastState = -1;

    /* ================= MECHANISM HARDWARE ================= */
    private DcMotorEx shooterMotor;
    private DcMotorEx intakeMotor;
    private DcMotorEx turretMotor;
    private Servo turretServo;

    /* ================= SHOOTER CONSTANTS (from TeleOp) ================= */
    private static final double SHOOTER_kP = 190.0;
    private static final double SHOOTER_kI = 13.0;
    private static final double SHOOTER_kD = 0.0;
    private static final double SHOOTER_kF = 18.0;

    /* ================= SHOT CONFIGURATION ARRAYS ================= */
    // Define RPM for each shot (index matches which shot you're on)
    private static final double[] SHOT_RPM = {
            1525.0,  // Shot 0 - Preload shot
            1450.0,  // Shot 1 - After pickup1
            1500.0,  // Shot 2 - After pickup2
            1525.0,  // Shot 3 - After pickup3
            1475.0,  // Shot 4 - Extra if needed
            1500.0   // Shot 5 - Extra if needed
    };

    // Define turret target ticks for each shot
    private static final int[] TURRET_TICKS = {
            500,   // Position 0 - Preload
            350,   // Position 1 - After pickup1
            450,   // Position 2 - After pickup2
            550,   // Position 3 - After pickup3
            400,   // Position 4 - Extra if needed
            500    // Position 5 - Extra if needed
    };

    /* ================= INTAKE CONSTANTS ================= */
    private static final double INTAKE_VELOCITY = 1800.0;

    /* ================= TURRET CONSTANTS ================= */
    private static final double TURRET_HOME = 0.45;
    private static final double TURRET_ACTIVE = 0.2;
    private static final double TURRET_POWER = 0.6;
    private static final double TURRET_HOLD_POWER_MAX = 0.15; // Maximum power to hold position
    private static final double TURRET_HOLD_kP = 0.01; // Proportional gain for holding
    private static final int TURRET_POSITION_TOLERANCE = 10; // Ticks tolerance for "at target"

    /* ================= TURRET STOPPER TIMING ================= */
    private static final double STOPPER_OPEN_TIME = 1.5; // seconds
    private static final double SHOOT_CYCLE_TIME = 3.0;  // Total time for shooting

    /* ================= STATE FLAGS ================= */
    private boolean turretMotorActive = false;
    private boolean turretReachedTarget = false;
    private int currentShotIndex = 0; // Track which shot we're on

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize follower
        follower = Constants.createFollower(hardwareMap,telemetry);
        follower.setStartingPose(new Pose(57, 8, Math.toRadians(90)));

        // Initialize mechanism hardware
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        turretServo = hardwareMap.get(Servo.class, "turretStopper");

        // Configure shooter motor with PIDF from TeleOp
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setVelocityPIDFCoefficients(
                SHOOTER_kP,
                SHOOTER_kI,
                SHOOTER_kD,
                SHOOTER_kF
        );

        // Configure intake motor
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Configure turret motor
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set turret servo to home position
        turretServo.setDirection(Servo.Direction.REVERSE);
        turretServo.setPosition(TURRET_HOME);

        // Build paths
        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized - Multi-Path Ready");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();

        // Update turret motor control
        updateTurretMotor();

        // Log telemetry
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Shot Index", currentShotIndex);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Shooter Target RPM", getCurrentShooterRPM());
        panelsTelemetry.debug("Shooter Actual RPM", shooterMotor.getVelocity());
        panelsTelemetry.debug("Turret Target Ticks", getCurrentTurretTicks());
        panelsTelemetry.debug("Turret Current Ticks", turretMotor.getCurrentPosition());
        panelsTelemetry.debug("Turret Reached", turretReachedTarget);
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain preLoadShot;
        public PathChain pickup1;
        public PathChain shoot1;
        public PathChain pickup2;
        public PathChain shoot2;
        public PathChain pickup3;
        public PathChain shoot3;

        public Paths(Follower follower) {
            preLoadShot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(57.000, 8.000),
                                    new Pose(67.000, 74.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            pickup1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(57.000, 85.000),
                                    new Pose(54.261, 55.717),
                                    new Pose(15.000, 65.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.000, 65.000),
                                    new Pose(57.000, 80.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            pickup2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(57.000, 80.000),
                                    new Pose(58.188, 25.940),
                                    new Pose(15.000, 36.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.000, 36.000),
                                    new Pose(57.000, 80.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            pickup3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(57.000, 80.000),
                                    new Pose(59.662, 1.338),
                                    new Pose(10.000, 15.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(10.000, 15.000),
                                    new Pose(55.000, 110.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }

    /**
     * Get the shooter RPM for the current shot
     */
    private double getCurrentShooterRPM() {
        if (currentShotIndex < SHOT_RPM.length) {
            return SHOT_RPM[currentShotIndex];
        }
        return SHOT_RPM[0]; // Default to first value if out of bounds
    }

    /**
     * Get the turret tick target for the current shot
     */
    private int getCurrentTurretTicks() {
        if (currentShotIndex < TURRET_TICKS.length) {
            return TURRET_TICKS[currentShotIndex];
        }
        return TURRET_TICKS[0]; // Default to first value if out of bounds
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            // ========== PRELOAD SHOT CYCLE ==========
            case 0: // Move to preload shot position
                shooterMotor.setVelocity(getCurrentShooterRPM());
                if (!turretMotorActive) {
                    turretMotorActive = true;
                    turretReachedTarget = false;
                }
                follow(paths.preLoadShot);
                advanceAfterPath();
                break;

            case 1: // Execute preload shot
                if (StateTimer.seconds() >= STOPPER_OPEN_TIME) {
                    turretServo.setPosition(TURRET_ACTIVE);
                    intakeMotor.setVelocity(INTAKE_VELOCITY);
                }
                advanceAfter(SHOOT_CYCLE_TIME);
                break;

            case 2: // Reset after preload shot
                turretServo.setPosition(TURRET_HOME);
                intakeMotor.setVelocity(0);
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                turretMotorActive = false;
                turretReachedTarget = false;
                currentShotIndex++; // Move to next shot configuration
                advance();
                break;

            // ========== PICKUP 1 + SHOOT CYCLE ==========
            case 3: // Move to pickup position 1
                follow(paths.pickup1);
                advanceAfterPath();
                break;

            case 4: // Move back to shooting position
                shooterMotor.setVelocity(getCurrentShooterRPM()); // New RPM for shot 1
                if (!turretMotorActive) {
                    turretMotorActive = true;
                    turretReachedTarget = false;
                }
                follow(paths.shoot1);
                advanceAfterPath();
                break;

            case 5: // Execute shot 1
                if (StateTimer.seconds() >= STOPPER_OPEN_TIME) {
                    turretServo.setPosition(TURRET_ACTIVE);
                    intakeMotor.setVelocity(INTAKE_VELOCITY);
                }
                advanceAfter(SHOOT_CYCLE_TIME);
                break;

            case 6: // Reset after shot 1
                turretServo.setPosition(TURRET_HOME);
                intakeMotor.setVelocity(0);
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                turretMotorActive = false;
                turretReachedTarget = false;
                currentShotIndex++; // Move to next shot configuration
                advance();
                break;

            // ========== PICKUP 2 + SHOOT CYCLE ==========
            case 7: // Move to pickup position 2
                follow(paths.pickup2);
                advanceAfterPath();
                break;

            case 8: // Move back to shooting position
                shooterMotor.setVelocity(getCurrentShooterRPM()); // New RPM for shot 2
                if (!turretMotorActive) {
                    turretMotorActive = true;
                    turretReachedTarget = false;
                }
                follow(paths.shoot2);
                advanceAfterPath();
                break;

            case 9: // Execute shot 2
                if (StateTimer.seconds() >= STOPPER_OPEN_TIME) {
                    turretServo.setPosition(TURRET_ACTIVE);
                    intakeMotor.setVelocity(INTAKE_VELOCITY);
                }
                advanceAfter(SHOOT_CYCLE_TIME);
                break;

            case 10: // Reset after shot 2
                turretServo.setPosition(TURRET_HOME);
                intakeMotor.setVelocity(0);
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                turretMotorActive = false;
                turretReachedTarget = false;
                currentShotIndex++; // Move to next shot configuration
                advance();
                break;

            // ========== PICKUP 3 + SHOOT CYCLE ==========
            case 11: // Move to pickup position 3
                follow(paths.pickup3);
                advanceAfterPath();
                break;

            case 12: // Move back to shooting position
                shooterMotor.setVelocity(getCurrentShooterRPM()); // New RPM for shot 3
                if (!turretMotorActive) {
                    turretMotorActive = true;
                    turretReachedTarget = false;
                }
                follow(paths.shoot3);
                advanceAfterPath();
                break;

            case 13: // Execute shot 3
                if (StateTimer.seconds() >= STOPPER_OPEN_TIME) {
                    turretServo.setPosition(TURRET_ACTIVE);
                    intakeMotor.setVelocity(INTAKE_VELOCITY);
                }
                advanceAfter(SHOOT_CYCLE_TIME);
                break;

            case 14: // Final cleanup
                turretServo.setPosition(TURRET_HOME);
                intakeMotor.setVelocity(0);
                shooterMotor.setVelocity(0);
                turretMotor.setPower(0);
                // Program complete
                break;

            default:
                // Safety stop
                shooterMotor.setVelocity(0);
                intakeMotor.setVelocity(0);
                turretMotor.setPower(0);
                break;
        }
        return pathState;
    }

    /**
     * Updates turret motor to move to target position during path following
     * Uses a holding power to prevent springback after reaching target
     */
    private void updateTurretMotor() {
        if (turretMotorActive && !turretReachedTarget) {
            int currentTicks = turretMotor.getCurrentPosition();
            int targetTicks = getCurrentTurretTicks();
            int error = targetTicks - currentTicks;

            // Check if target reached (with small tolerance)
            if (Math.abs(error) < TURRET_POSITION_TOLERANCE) {
                turretReachedTarget = true;
            } else {
                // Move toward target with proportional power for smoother approach
                double power = TURRET_POWER;

                // Slow down as we approach target (proportional control)
                if (Math.abs(error) < 100) {
                    power = 0.3 + (Math.abs(error) / 100.0) * (TURRET_POWER - 0.3);
                }

                if (error > 0) {
                    turretMotor.setPower(power);
                } else {
                    turretMotor.setPower(-power);
                }
            }
        }

        // Hold position when target reached to prevent springback
        if (turretReachedTarget && turretMotorActive) {
            int currentTicks = turretMotor.getCurrentPosition();
            int targetTicks = getCurrentTurretTicks();
            int error = targetTicks - currentTicks;

            // Use a small holding power to maintain position
            double holdPower = error * TURRET_HOLD_kP; // Proportional holding
            holdPower = Math.max(-TURRET_HOLD_POWER_MAX, Math.min(TURRET_HOLD_POWER_MAX, holdPower)); // Limit holding power

            turretMotor.setPower(holdPower);
        }

        if (!turretMotorActive) {
            turretMotor.setPower(0);
        }
    }

    private void follow(PathChain path) {
        follow(path, false);
    }

    public void follow(PathChain path, boolean holdPoint) {
        if (pathState != lastState) {
            follower.followPath(path, holdPoint);
            lastState = pathState;
        }
    }

    private void advance() {
        pathState++;
        StateTimer.reset();
        isWaiting = false;
    }

    private void advanceWhen(boolean cond) {
        if (cond) advance();
    }

    private void advanceAfterPath() {
        advanceWhen(!follower.isBusy());
    }

    private void advanceAfterPathWithMin(double minimumTime) {
        if (StateTimer.seconds() > minimumTime)
            advanceAfterPath();
    }

    private void advanceAfter(double seconds) {
        if (!isWaiting) advanceTimer.reset();
        isWaiting = true;
        advanceWhen(advanceTimer.seconds() > seconds);
        if (advanceTimer.seconds() > seconds)
            isWaiting = false;
    }
}
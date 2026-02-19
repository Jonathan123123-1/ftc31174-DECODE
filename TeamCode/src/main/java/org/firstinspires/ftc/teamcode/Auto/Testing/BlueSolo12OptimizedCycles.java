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


@Autonomous(name = "Blue 12 Optimized Gate Cycles", group = "Autonomous")
@Configurable
public class BlueSolo12OptimizedCycles extends OpMode {
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
    private static final double SHOOTER_kP = 225.0;
    private static final double SHOOTER_kI = 11.0;
    private static final double SHOOTER_kD = 0.0;
    private static final double SHOOTER_kF = 14.0;

    /* ================= SHOT CONFIGURATION ARRAYS ================= */
    // Define RPM for each shot (now includes gate cycle shots)
    private static final double[] SHOT_RPM = {
            1260.0,  // Preload shot
            1260.0,  // Gate cycle 1 shot
            1260.0,  // Gate cycle 2 shot
            1260.0,  // Gate cycle 3 shot
            1260.0,  // Pickup 2 shot
            1235.0,  // Pickup 3 shot (second to last)
            1500.0   // Final shot
    };

    // Define turret target ticks for each shot
    private static final int[] TURRET_TICKS = {
            -206,   // Preload position
            -1,     // Gate cycle 1 position
            -1,     // Gate cycle 2 position
            -4,    // Gate cycle 3 position
            122,    // Pickup 2 position
            125,    // Pickup 3 position
            500     // Final position
    };

    /* ================= INTAKE CONSTANTS ================= */
    private static final double INTAKE_VELOCITY = 1800.0;

    /* ================= TURRET CONSTANTS ================= */
    private static final double TURRET_HOME = 0.45;
    private static final double TURRET_ACTIVE = 0.2;
    private static final double TURRET_POWER = 0.6;
    private static final double TURRET_HOLD_POWER_MAX = 0.15;
    private static final double TURRET_HOLD_kP = 0.008;
    private static final int TURRET_POSITION_TOLERANCE = 10;

    /* ================= TURRET STOPPER TIMING ================= */
    private static final double STOPPER_OPEN_TIME = 1.3;

    /* ================= GATE CYCLE TIMING ================= */
    private static final double GATE_OPEN_TIME = 0.5; // Time to just open gate without intake
    private static final double PICKUP_AFTER_GATE_TIME = 0.9; // Time to pickup after moving down from gate

    /* ================= STATE FLAGS ================= */
    private boolean turretMotorActive = false;
    private boolean turretReachedTarget = false;
    private int currentShotIndex = 0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize follower
        follower = Constants.createFollower(hardwareMap,telemetry);
        follower.setStartingPose(new Pose(33, 134.5, Math.toRadians(270)));

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

        panelsTelemetry.debug("Status", "Optimized Gate Cycles Initialized");
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

        // Gate cycle paths (3 cycles)
        public PathChain gateApproach1;
        public PathChain gatePickup1;
        public PathChain gateShoot1;

        public PathChain gateApproach2;
        public PathChain gatePickup2;
        public PathChain gateShoot2;

        public PathChain gateApproach3;
        public PathChain gatePickup3;
        public PathChain gateShoot3;

        // Original pickup paths (still used after gate cycles)
        public PathChain pickup2;
        public PathChain shoot2;
        public PathChain pickup3;
        public PathChain shoot3;

        public Paths(Follower follower) {
            // Preload shot path
            preLoadShot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(33.000, 134.500),
                                    new Pose(67.000, 73.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                    .build();

            // ========== GATE CYCLE 1 ==========
            // Approach gate (opens gate, NO intake during this path)
            gateApproach1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(67.000, 73.000),
                                    new Pose(54.261, 55.717),
                                    new Pose(19.580, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(153))
                    .build();

            // Move down to pickup artifacts that fell through gate (with intake ON)
            gatePickup1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.580, 60.000),
                                    new Pose(18.200, 54.000) // Moved down ~8 inches
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(153))
                    .build();

            // Return to shoot
            gateShoot1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.200, 54.000),
                                    new Pose(67.000, 73.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            // ========== GATE CYCLE 2 ==========
            // Approach gate again (opens gate, NO intake)
            gateApproach2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(67.000, 73.000),
                                    new Pose(54.261, 55.717),
                                    new Pose(19.580, 60.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(153))
                    .build();

            // Move down to pickup (with intake ON)
            gatePickup2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.580, 60.000),
                                    new Pose(18.200, 54.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(153))
                    .build();

            // Return to shoot
            gateShoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.200, 54.000),
                                    new Pose(67.000, 73.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            // ========== GATE CYCLE 3 ==========
            // Approach gate again (opens gate, NO intake)
            gateApproach3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(67.000, 73.000),
                                    new Pose(54.261, 55.717),
                                    new Pose(19.580, 60.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(153))
                    .build();

            // Move down to pickup (with intake ON)
            gatePickup3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.580, 60.000),
                                    new Pose(18.200, 53.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(153))
                    .build();

            // Return to shoot
            gateShoot3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.200, 53.000),
                                    new Pose(64.000, 73.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            // ========== ORIGINAL PICKUP 2 & 3 (after gate cycles) ==========
            pickup2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(64.000, 73.000),
                                    new Pose(48.188, 25.940),
                                    new Pose(20.000, 37.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20.000, 37.000),
                                    new Pose(64.000, 73.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            pickup3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(64.000, 73.000),
                                    new Pose(44.500, 83.500),
                                    new Pose(24.420, 82.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.420, 82.000),
                                    new Pose(60.000, 110.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }

    private double getCurrentShooterRPM() {
        if (currentShotIndex < SHOT_RPM.length) {
            return SHOT_RPM[currentShotIndex];
        }
        return SHOT_RPM[0];
    }

    private int getCurrentTurretTicks() {
        if (currentShotIndex < TURRET_TICKS.length) {
            return TURRET_TICKS[currentShotIndex];
        }
        return TURRET_TICKS[0];
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            // ========== PRELOAD SHOT CYCLE ==========
            case 0: // preLoadShot path - Start shooter and turret
                shooterMotor.setVelocity(getCurrentShooterRPM());
                if (!turretMotorActive) {
                    turretMotorActive = true;
                    turretReachedTarget = false;
                }
                follow(paths.preLoadShot);
                advanceAfterPath();
                break;

            case 1: // Shooting sequence - stopper and intake
                if (StateTimer.seconds() >= STOPPER_OPEN_TIME) {
                    turretServo.setPosition(TURRET_ACTIVE);
                    intakeMotor.setVelocity(-INTAKE_VELOCITY);
                }
                advanceAfter(2.1);
                break;

            case 2: // Reset after preload shot
                turretServo.setPosition(TURRET_HOME);
                intakeMotor.setVelocity(0);
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                turretMotorActive = false;
                turretReachedTarget = false;
                currentShotIndex++;
                advance();
                break;

            // ========== GATE CYCLE 1 ==========
            case 3: // Approach gate to open it (NO intake yet)
                // DON'T run intake - just open the gate by driving to it
                intakeMotor.setVelocity(-INTAKE_VELOCITY);
                follow(paths.gateApproach1);
                advanceAfterPath();
                break;

            case 4: // Wait briefly for gate to open
                advanceAfter(GATE_OPEN_TIME);
                break;

            case 5: // Stop intake
                intakeMotor.setVelocity(0);
                advance();
                break;

            case 6: // Return to shoot with new settings
                shooterMotor.setVelocity(getCurrentShooterRPM());
                if (!turretMotorActive) {
                    turretMotorActive = true;
                    turretReachedTarget = false;
                }
                follow(paths.gateShoot1);
                advanceAfterPath();
                break;

            case 7: // Execute shot
                if (StateTimer.seconds() >= STOPPER_OPEN_TIME) {
                    turretServo.setPosition(TURRET_ACTIVE);
                    intakeMotor.setVelocity(-INTAKE_VELOCITY);
                }
                advanceAfter(2.1);
                break;

            case 8: // Reset after shot
                turretServo.setPosition(TURRET_HOME);
                intakeMotor.setVelocity(0);
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                turretMotorActive = false;
                turretReachedTarget = false;
                currentShotIndex++;
                advance();
                break;

            // ========== GATE CYCLE 2 ==========
            case 9: // Approach gate (NO intake)
                intakeMotor.setVelocity(-INTAKE_VELOCITY);
                follow(paths.gateApproach2);
                advanceAfterPath();
                break;

            case 10: // Wait for gate
                advanceAfter(GATE_OPEN_TIME);
                break;

            case 11: // Move down and pickup
                intakeMotor.setVelocity(-INTAKE_VELOCITY);
                follow(paths.gatePickup2);
                advanceAfterPath();
                break;

            case 12: // Continue intake
                advanceAfter(PICKUP_AFTER_GATE_TIME);
                break;

            case 13: // Stop intake
                intakeMotor.setVelocity(0);
                advance();
                break;

            case 14: // Return to shoot
                shooterMotor.setVelocity(getCurrentShooterRPM());
                if (!turretMotorActive) {
                    turretMotorActive = true;
                    turretReachedTarget = false;
                }
                follow(paths.gateShoot2);
                advanceAfterPath();
                break;

            case 15: // Execute shot
                if (StateTimer.seconds() >= STOPPER_OPEN_TIME) {
                    turretServo.setPosition(TURRET_ACTIVE);
                    intakeMotor.setVelocity(-INTAKE_VELOCITY);
                }
                advanceAfter(2.1);
                break;

            case 16: // Reset after shot
                turretServo.setPosition(TURRET_HOME);
                intakeMotor.setVelocity(0);
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                turretMotorActive = false;
                turretReachedTarget = false;
                currentShotIndex++;
                advance();
                break;

            // ========== PICKUP 2 (Original pattern resumes) ==========
            case 17: // Move to pickup position 2 and intake
                intakeMotor.setVelocity(-INTAKE_VELOCITY);
                follow(paths.pickup3);
                advanceAfterPath();
                break;

            case 18: // Stop intake
                intakeMotor.setVelocity(0);
                advance();
                break;

            case 19: // Return to shoot
                shooterMotor.setVelocity(getCurrentShooterRPM());
                if (!turretMotorActive) {
                    turretMotorActive = true;
                    turretReachedTarget = false;
                }
                follow(paths.shoot3);
                advanceAfterPath();
                break;

            case 20: // Execute shot
                if (StateTimer.seconds() >= STOPPER_OPEN_TIME) {
                    turretServo.setPosition(TURRET_ACTIVE);
                    intakeMotor.setVelocity(-INTAKE_VELOCITY);
                }
                advanceAfter(2.1);
                break;

            case 21: // Reset
                turretServo.setPosition(TURRET_HOME);
                intakeMotor.setVelocity(0);
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                turretMotorActive = false;
                turretReachedTarget = false;
                currentShotIndex++;
                advance();
                break;

            // ========== PICKUP 3 (Final cycle) ==========
            case 22: // Move to pickup position 3
                intakeMotor.setVelocity(-INTAKE_VELOCITY);
                follow(paths.pickup3);
                advanceAfterPath();
                break;

            case 23: // Stop intake
                intakeMotor.setVelocity(0);
                advance();
                break;

            case 24: // FINAL - Stop everything
                turretServo.setPosition(TURRET_HOME);
                intakeMotor.setVelocity(0);
                shooterMotor.setVelocity(0);
                turretMotor.setPower(0);
                break;

            default:
                shooterMotor.setVelocity(0);
                intakeMotor.setVelocity(0);
                turretMotor.setPower(0);
                break;
        }
        return pathState;
    }

    private void updateTurretMotor() {
        if (turretMotorActive && !turretReachedTarget) {
            int currentTicks = turretMotor.getCurrentPosition();
            int targetTicks = getCurrentTurretTicks();
            int error = targetTicks - currentTicks;

            if (Math.abs(error) < TURRET_POSITION_TOLERANCE) {
                turretReachedTarget = true;
            } else {
                double power = TURRET_POWER;

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

        if (turretReachedTarget && turretMotorActive) {
            int currentTicks = turretMotor.getCurrentPosition();
            int targetTicks = getCurrentTurretTicks();
            int error = targetTicks - currentTicks;

            double holdPower = error * TURRET_HOLD_kP;
            holdPower = Math.max(-TURRET_HOLD_POWER_MAX, Math.min(TURRET_HOLD_POWER_MAX, holdPower));

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
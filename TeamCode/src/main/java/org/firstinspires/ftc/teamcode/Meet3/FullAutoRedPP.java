package org.firstinspires.ftc.teamcode.Meet3;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "100% - 12 Ball red", group = "Autonomous")
@Configurable
public class FullAutoRedPP extends OpMode {

    // --- State Machine Enums ---
    private enum AutoState {
        START,
        PRELOAD_AIM,
        PRELOAD_SHOOT,
        PRELOAD_RESET,
        PATH_FOLLOWING,
        SHOOT_1_AIM,
        SHOOT_1_SHOOT,
        SHOOT_1_RESET,
        SHOOT_2_AIM,
        SHOOT_2_SHOOT,
        SHOOT_2_RESET,
        SHOOT_3_AIM,
        SHOOT_3_SHOOT,
        SHOOT_3_RESET,
        DONE
    }

    private enum ShootingCycleState {
        IDLE, START, STOPPER_UP, INTAKE_PULSE, STOPPER_DOWN, CHECK_CYCLE
    }

    // --- Constants ---
    private static final double SHOOTER_VELOCITY_RPM = 1570;
    private static final double INTAKE_VELOCITY = 3500; // From odometryBlue
    private static final int TURRET_PRELOAD_TICKS = -63;
    private static final int TURRET_SHOOT_TICKS = 331;
    private static final double INTAKE_PULSE_SECONDS = 1.0;
    private static final int SHOOT_CYCLE_REPETITIONS = 3;
    private static final double STOPPER_UP_POS = 0.5; // TUNE THIS VALUE
    private static final double STOPPER_DOWN_POS = 0.2; // TUNE THIS VALUE
    // TUNE THESE PIDF VALUES FOR YOUR SHOOTER MOTOR
    private static final PIDFCoefficients SHOOTER_PIDF = new PIDFCoefficients(20, 0, 5, 14.5);

    // --- Hardware ---
    private Follower follower;
    private DcMotorEx turretMotor, shooterMotor, intakeMotor;
    private Servo turretStopper;

    // --- State Management ---
    private AutoState currentState = AutoState.START;
    private ShootingCycleState shootingState = ShootingCycleState.IDLE;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private int shotCycleCounter = 0;
    private PathChain lastPathStarted = null;

    // --- Paths & Telemetry ---
    private Paths paths;
    private TelemetryManager panelsTelemetry;

    @Override
    public void init() {
        // Telemetry
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Hardware Mapping (THESE NAMES MUST MATCH YOUR CONFIGURATION)
        follower = Constants.createFollower(hardwareMap);
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        turretStopper = hardwareMap.get(Servo.class, "turretStopper");

        // Motor & Servo Configuration
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.8); // Power for turret position movements

        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE); // May need to change
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SHOOTER_PIDF);

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Changed for velocity control

        turretStopper.setPosition(STOPPER_DOWN_POS);

        // Pathing
        follower.setStartingPose(new Pose(87, 8, Math.toRadians(90)));
        paths = new Paths(follower);

        // Pre-heat shooter motor to target speed during init
        shooterMotor.setVelocity(SHOOTER_VELOCITY_RPM);

        panelsTelemetry.debug("Status", "Initialized and Pre-heating Shooter");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        stateTimer.reset();
        transitionToState(AutoState.PRELOAD_AIM);
    }

    @Override
    public void loop() {
        follower.update(); // Essential for path following
        runStateMachine();

        // Constantly update telemetry
        panelsTelemetry.debug("Main State", currentState);
        panelsTelemetry.debug("Shooting Sub-State", shootingState);
        panelsTelemetry.debug("Turret Ticks", turretMotor.getCurrentPosition());
        panelsTelemetry.debug("Shooter RPM", shooterMotor.getVelocity());
        panelsTelemetry.debug("Follower IsBusy", follower.isBusy());
        panelsTelemetry.update(telemetry);
    }

    private void runStateMachine() {
        switch (currentState) {
            case PRELOAD_AIM:
                turretMotor.setTargetPosition(TURRET_PRELOAD_TICKS);
                if (!turretMotor.isBusy()) { // Wait for turret to reach position
                    transitionToState(AutoState.PRELOAD_SHOOT);
                }
                break;

            case PRELOAD_SHOOT:
                runShootingCycle(AutoState.PRELOAD_RESET); // Run 3-shot burst
                break;

            case PRELOAD_RESET:
                turretMotor.setTargetPosition(0);
                if (!turretMotor.isBusy()) { // Wait for turret to reset
                    follower.followPath(paths.preLoadShot, true);
                    lastPathStarted = paths.preLoadShot;
                    transitionToState(AutoState.PATH_FOLLOWING);
                }
                break;

            case PATH_FOLLOWING:
                if (!follower.isBusy()) { // This block only runs when a path is COMPLETED
                    // Default to turning intake off after a path unless specified otherwise
                    intakeMotor.setVelocity(0);

                    if (lastPathStarted == paths.preLoadShot) {
                        intakeMotor.setVelocity(-INTAKE_VELOCITY); // Turn intake on for next two paths
                        follower.followPath(paths.pickup1, true);
                        lastPathStarted = paths.pickup1;
                    } else if (lastPathStarted == paths.pickup1) {
                        intakeMotor.setVelocity(-INTAKE_VELOCITY); // Keep intake on
                        follower.followPath(paths.openGate1, true);
                        lastPathStarted = paths.openGate1;
                    } else if (lastPathStarted == paths.openGate1) {
                        transitionToState(AutoState.SHOOT_1_AIM); // Arrived at shooting spot

                    } else if (lastPathStarted == paths.shoot1) {
                        intakeMotor.setVelocity(-INTAKE_VELOCITY); // Turn intake on for pickup
                        follower.followPath(paths.pickup2, true);
                        lastPathStarted = paths.pickup2;
                    } else if (lastPathStarted == paths.pickup2) {
                        follower.followPath(paths.grab1, true); // Intake is off for this path
                        lastPathStarted = paths.grab1;
                    } else if (lastPathStarted == paths.grab1) {
                        intakeMotor.setVelocity(-INTAKE_VELOCITY); // Intake ON to secure balls before moving
                        follower.followPath(paths.shoot2, true);
                        lastPathStarted = paths.shoot2;
                    } else if (lastPathStarted == paths.shoot2) {
                        transitionToState(AutoState.SHOOT_2_AIM);

                    } else if (lastPathStarted == paths.grab2) {
                        intakeMotor.setVelocity(-INTAKE_VELOCITY); // Keep intake on
                        follower.followPath(paths.shoot3, true);
                        lastPathStarted = paths.shoot3;
                    } else if (lastPathStarted == paths.shoot3) {
                        transitionToState(AutoState.SHOOT_3_AIM);

                    } else if (lastPathStarted == paths.getOut1) {
                        transitionToState(AutoState.DONE); // End of auto
                    }
                }
                break;

            case SHOOT_1_AIM:
                turretMotor.setTargetPosition(TURRET_SHOOT_TICKS);
                if (!turretMotor.isBusy()) {
                    transitionToState(AutoState.SHOOT_1_SHOOT);
                }
                break;

            case SHOOT_1_SHOOT:
                runShootingCycle(AutoState.SHOOT_1_RESET);
                break;

            case SHOOT_1_RESET:
                turretMotor.setTargetPosition(0);
                if (!turretMotor.isBusy()) {
                    follower.followPath(paths.shoot1, true);
                    lastPathStarted = paths.shoot1;
                    transitionToState(AutoState.PATH_FOLLOWING);
                }
                break;

            case SHOOT_2_AIM:
                turretMotor.setTargetPosition(TURRET_SHOOT_TICKS);
                if (!turretMotor.isBusy()) {
                    transitionToState(AutoState.SHOOT_2_SHOOT);
                }
                break;

            case SHOOT_2_SHOOT:
                runShootingCycle(AutoState.SHOOT_2_RESET);
                break;

            case SHOOT_2_RESET:
                turretMotor.setTargetPosition(0);
                if (!turretMotor.isBusy()) {
                    intakeMotor.setVelocity(-INTAKE_VELOCITY); // Turn on intake for the next path
                    follower.followPath(paths.grab2, true);
                    lastPathStarted = paths.grab2;
                    transitionToState(AutoState.PATH_FOLLOWING);
                }
                break;

            case SHOOT_3_AIM:
                turretMotor.setTargetPosition(TURRET_SHOOT_TICKS);
                if (!turretMotor.isBusy()) {
                    transitionToState(AutoState.SHOOT_3_SHOOT);
                }
                break;

            case SHOOT_3_SHOOT:
                runShootingCycle(AutoState.SHOOT_3_RESET);
                break;

            case SHOOT_3_RESET:
                turretMotor.setTargetPosition(0);
                if (!turretMotor.isBusy()) {
                    follower.followPath(paths.getOut1, true);
                    lastPathStarted = paths.getOut1;
                    transitionToState(AutoState.PATH_FOLLOWING);
                }
                break;

            case DONE:
                // Stop all motion
                shooterMotor.setVelocity(0);
                intakeMotor.setVelocity(0);
                turretMotor.setPower(0); // Cut power to lock turret in place with BRAKE behavior
                break;
        }
    }

    private void runShootingCycle(AutoState nextStateAfterDone) {
        switch (shootingState) {
            case IDLE:
                shotCycleCounter = 0;
                shootingState = ShootingCycleState.START;
                break;
            case START:
                if (shotCycleCounter < SHOOT_CYCLE_REPETITIONS) {
                    shootingState = ShootingCycleState.STOPPER_UP;
                } else {
                    transitionToState(nextStateAfterDone); // Finished all repetitions
                }
                break;
            case STOPPER_UP:
                turretStopper.setPosition(STOPPER_UP_POS);
                stateTimer.reset();
                shootingState = ShootingCycleState.INTAKE_PULSE;
                break;
            case INTAKE_PULSE:
                intakeMotor.setVelocity(-INTAKE_VELOCITY); // Pulse intake to feed ball
                if (stateTimer.seconds() > INTAKE_PULSE_SECONDS) {
                    intakeMotor.setVelocity(0);
                    shootingState = ShootingCycleState.STOPPER_DOWN;
                }
                break;
            case STOPPER_DOWN:
                turretStopper.setPosition(STOPPER_DOWN_POS);
                shotCycleCounter++;
                // Add a small delay for servo to move before starting next cycle
                if (stateTimer.seconds() > INTAKE_PULSE_SECONDS + 0.3) {
                    shootingState = ShootingCycleState.START;
                }
                break;
        }
    }

    // Helper method to manage state transitions and reset sub-states
    private void transitionToState(AutoState nextState) {
        if (currentState != nextState) {
            currentState = nextState;
            stateTimer.reset();
            // When we move to a new main state, always reset the shooting sub-state to IDLE
            shootingState = ShootingCycleState.IDLE;
        }
    }

    // YOUR PATHS ARE UNTOUCHED, AS REQUESTED
    public static class Paths {
        public PathChain preLoadShot, pickup1, openGate1, shoot1, pickup2, grab1, shoot2, grab2, shoot3, getOut1;

        // Custom constraint levels
        private final PathConstraints slowConstraints =
                new PathConstraints(10, 2, Math.toRadians(180), Math.toRadians(180));

        public Paths(Follower follower) {
            // preLoadShot
            preLoadShot = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(87.000, 8.000),
                            new Pose(88.000, 36.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            // pickup1 first 3
            pickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(88.000, 36.000),
                            new Pose(121.000, 36.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // shoot1
            openGate1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(121.000, 36.000),
                            new Pose(84.000, 10.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // pickup2 for the next 3
            shoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(134.000, 10.000),
                            new Pose(88.000, 56.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // grab1 for the balls
            pickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(88.000, 56.000),
                            new Pose(119.000, 56.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // opening the gate
            grab1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(119.000, 56.000),
                            new Pose(114.000, 60.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                    .build();

            // shoot2
            shoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(114.000, 60.000),
                            new Pose(84.000, 10.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))
                    .build();

            // pickup2
            grab2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(84.000, 10.000),
                            new Pose(121.000, 10.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // shoot again
            shoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(121.000, 10.000),
                            new Pose(84.000, 10.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // get out
            getOut1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(84.000, 10.000),
                            new Pose(120.000, 15.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }
    }
}
package org.firstinspires.ftc.teamcode.shooterConstants;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class fusedLocalizer implements Localizer {

    private final PinpointLocalizer odom;
    private final LimelightVision limelight;
    private final Telemetry telemetry;

    // === TUNABLE ===
    private static final double VISION_WEIGHT = 0.05; // 0.05–0.15 recommended

    private Pose currentPose = new Pose(0, 0, 0);

    public fusedLocalizer(
            PinpointLocalizer odom,
            LimelightVision limelight,
            Telemetry telemetry
    ) {
        this.odom = odom;
        this.limelight = limelight;
        this.telemetry = telemetry;
    }

    @Override
    public void update() {
        // Always update odometry
        odom.update();
        Pose odomPose = odom.getPose();
        currentPose = odomPose;

        boolean visionUsed = false;

        if (limelight.hasPose()) {
            Pose visionPose = limelight.getFieldPose();

            double fusedX = lerp(odomPose.getX(), visionPose.getX(), VISION_WEIGHT);
            double fusedY = lerp(odomPose.getY(), visionPose.getY(), VISION_WEIGHT);
            double fusedH = angleLerp(
                    odomPose.getHeading(),
                    visionPose.getHeading(),
                    VISION_WEIGHT
            );

            currentPose = new Pose(fusedX, fusedY, fusedH);
            odom.setPose(currentPose);
            visionUsed = true;
        }

        if (telemetry != null) {
            telemetry.addData("Vision", visionUsed ? "USED" : "NO POSE");
            telemetry.addData("X", currentPose.getX());
            telemetry.addData("Y", currentPose.getY());
            telemetry.addData("H (deg)", Math.toDegrees(currentPose.getHeading()));
        }
    }

    @Override
    public Pose getPose() {
        return currentPose;
    }

    @Override
    public Pose getVelocity() {
        return odom.getVelocity();
    }

    @Override
    public Vector getVelocityVector() {
        return getVelocity().getAsVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        odom.setStartPose(setStart);
        currentPose = setStart;
    }

    @Override
    public void setPose(Pose setPose) {
        odom.setPose(setPose);
        currentPose = setPose;
    }

    @Override
    public double getTotalHeading() {
        return odom.getTotalHeading();
    }

    @Override
    public double getForwardMultiplier() {
        return odom.getForwardMultiplier();
    }

    @Override
    public double getLateralMultiplier() {
        return odom.getLateralMultiplier();
    }

    @Override
    public double getTurningMultiplier() {
        return odom.getTurningMultiplier();
    }

    @Override
    public void resetIMU() {}

    @Override
    public double getIMUHeading() {
        return Double.NaN;
    }

    @Override
    public boolean isNAN() {
        return Double.isNaN(currentPose.getX())
                || Double.isNaN(currentPose.getY())
                || Double.isNaN(currentPose.getHeading());
    }

    // ================= HELPERS =================

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private static double angleLerp(double a, double b, double t) {
        double diff = Math.atan2(Math.sin(b - a), Math.cos(b - a));
        return a + diff * t;
    }
}
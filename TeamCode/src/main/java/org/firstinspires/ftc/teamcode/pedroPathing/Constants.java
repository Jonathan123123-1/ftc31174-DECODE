package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(6.7)
            .forwardZeroPowerAcceleration(-138.01028229133647)
            .lateralZeroPowerAcceleration(-726.1591685415265);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("front_right_drive")
            .rightRearMotorName("back_right_drive")
            .leftRearMotorName("back_left_drive")
            .leftFrontMotorName("front_left_drive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(150.33392279666597)
            .yVelocity(355.3174396683708);

    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .robotWidth(18)
            .robotLength(18)
            .rightFrontMotorName("front_right_drive")
            .rightRearMotorName("back_right_drive")
            .leftRearMotorName("back_left_drive")
            .leftFrontMotorName("front_left_drive")
            .leftFrontEncoderDirection(Encoder.REVERSE)
            .leftRearEncoderDirection(Encoder.REVERSE)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.FORWARD)
            .forwardTicksToInches(0.006502899247729156)
            .strafeTicksToInches(0.007454636063004041)
            .turnTicksToInches(0.022104433798345147);

    public static PathConstraints pathConstraints = new PathConstraints(
            35, // maxVel
            35,  // maxAccel
            Math.toRadians(180), // maxAngVel (radians/sec)
            Math.toRadians(180)
    );

    public static com.pedropathing.follower.Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .driveEncoderLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
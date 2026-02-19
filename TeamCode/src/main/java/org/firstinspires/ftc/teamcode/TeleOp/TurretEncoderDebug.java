package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Turret Encoder Debug", group = "DEBUG")
public class TurretEncoderDebug extends OpMode {

    private DcMotorEx turretMotor;

    private static final double TURRET_POWER = 0.4;

    @Override
    public void init() {

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Turret Encoder Debug Ready");
        telemetry.update();
    }

    @Override
    public void loop() {

        if (gamepad2.left_bumper) {
            turretMotor.setPower(-TURRET_POWER);
        } else if (gamepad2.right_bumper) {
            turretMotor.setPower(TURRET_POWER);
        } else {
            turretMotor.setPower(0);
        }

        telemetry.addData("Turret Encoder Ticks", turretMotor.getCurrentPosition());
        telemetry.update();
    }
}

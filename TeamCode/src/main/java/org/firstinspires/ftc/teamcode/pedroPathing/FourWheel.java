package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "4 wheel drive")
public class FourWheel extends OpMode {
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    DcMotorEx front_left;
    DcMotorEx front_right;
    DcMotorEx back_left;
    DcMotorEx back_right;
    boolean useMax = false;
    double driveSpeed;

    enum Speed {
        MAX,
        MED,
        MIN
    }

    Speed driveSpeedEnum = Speed.MED;

    @Override
    public void init() {
        front_left  = hardwareMap.get(DcMotorEx.class, "frontleft");
        front_right = hardwareMap.get(DcMotorEx.class, "frontright");
        back_left   = hardwareMap.get(DcMotorEx.class, "backleft");
        back_right  = hardwareMap.get(DcMotorEx.class, "backright");

        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetryM.addData("Initialized", true);
        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {
        telemetryM.addLine("X to change speed, A to toggle max");

        if (gamepad1.x) {
            switch (driveSpeedEnum) {
                case MAX:
                    driveSpeedEnum = Speed.MED;
                    driveSpeed = 0.5;
                    break;
                case MED:
                    driveSpeedEnum = Speed.MIN;
                    driveSpeed = 0.25;
                    break;
                case MIN:
                    driveSpeedEnum = Speed.MAX;
                    driveSpeed = 1;
                    break;
            }
        }

        if (gamepad1.a) useMax = !useMax;

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        double max = 1;
        double flPower = (y + x + rx) * driveSpeed;
        double frPower = (y - x - rx) * driveSpeed;
        double blPower = (y - x + rx) * driveSpeed;
        double brPower = (y + x - rx) * driveSpeed;

        max = Math.max(Math.abs(Math.max(Math.max(Math.abs(frPower), Math.abs(flPower)), Math.abs(blPower))), Math.abs(brPower));

        telemetryM.addData("Current speed setting", driveSpeedEnum.toString());
        telemetryM.addData("Current speed value", driveSpeed);

        if (useMax && max > 1) {
            flPower /= max;
            frPower /= max;
            blPower /= max;
            brPower /= max;

            telemetryM.addData("max value", max);
        }

        front_left.setPower(flPower);
        front_right.setPower(frPower);
        back_left.setPower(blPower);
        back_right.setPower(brPower);

        telemetryM.update(telemetry);
    }
}

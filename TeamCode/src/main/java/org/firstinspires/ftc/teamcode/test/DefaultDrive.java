package org.firstinspires.ftc.teamcode.test;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "Default Drive", group = "TeleOp")
public class DefaultDrive extends OpMode {
    DcMotorEx front_left;
    DcMotorEx front_right;
    DcMotorEx back_left;
    DcMotorEx back_right;
    double driveSpeed = 0.5;

    enum Speed {
        MAX,
        MED,
        ALM,
        MIN
    }

    Speed driveSpeedEnum = Speed.MED;

    @Override
    public void init() {
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        front_left  = hardwareMap.get(DcMotorEx.class, "frontleft");
        front_right = hardwareMap.get(DcMotorEx.class, "frontright");
        back_left   = hardwareMap.get(DcMotorEx.class, "backleft");
        back_right  = hardwareMap.get(DcMotorEx.class, "backright");


        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_left.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior  (DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);

        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Initialized", true);
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addLine("right bumper to change speed");

        if (gamepad1.rightBumperWasPressed()) {
            switch (driveSpeedEnum) {
                case MAX:
                    driveSpeedEnum = Speed.ALM;
                    driveSpeed = 0.75;
                    break;
                case ALM:
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

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        double max;
        double flPower = (y + x + rx) * driveSpeed;
        double frPower = (y - x - rx) * driveSpeed;
        double blPower = (y - x + rx) * driveSpeed;
        double brPower = (y + x - rx) * driveSpeed;

        max = Math.max(Math.abs(Math.max(Math.max(Math.abs(frPower), Math.abs(flPower)), Math.abs(blPower))), Math.abs(brPower));

        telemetry.addData("Current speed setting", driveSpeedEnum.toString());
        telemetry.addData("Current speed value", driveSpeed);

        if (max > 1) {
            flPower /= max;
            frPower /= max;
            blPower /= max;
            brPower /= max;
        }

        front_left.setPower(flPower);
        front_right.setPower(frPower);
        back_left.setPower(blPower);
        back_right.setPower(brPower);

        if (gamepad1.a)
            back_left.setPower(driveSpeed);
        if (gamepad1.x)
            front_left.setPower(driveSpeed);
        if (gamepad1.y)
            front_right.setPower(driveSpeed);
        if (gamepad1.b)
            back_right.setPower(driveSpeed);

        telemetry.addLine("---------------------------------------");
        telemetry.addData("A", "back_left");
        telemetry.addData("X", "front_left");
        telemetry.addData("Y", "front_right");
        telemetry.addData("B", "back_right");

        telemetry.update();
    }
}

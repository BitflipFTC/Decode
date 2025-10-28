package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp()
public class GamepadOpMode extends OpMode {
    @Override
    public void init() {
        DcMotor frontLeftDrive;
        DcMotor frontright;
        DcMotor backleft;
        DcMotor backright;

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.FORWARD);
        backright.setDirection(DcMotorSimple.Direction.FORWARD);

        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        double frontleftpower = axial + lateral + yaw;
        double frontrightpower = axial - lateral - yaw;
        double backleftpower = axial - lateral + yaw;
        double backrightpower = axial + lateral - yaw;
    }

    @Override
    public void loop() {
        telemetry.addData("Left stick x", gamepad1.left_stick_x);
        telemetry.addData("Left stick y", -gamepad1.left_stick_y);
        telemetry.addData("A button", gamepad1.a);
        telemetry.addData("Right stick x", gamepad1.right_stick_x);
        telemetry.addData("Right stick y", -gamepad1.right_stick_y);
        telemetry.addData("B button", gamepad1.b);
        telemetry.addData("Left trigger", gamepad1.left_trigger);
        telemetry.addData("Right trigger", gamepad1.right_trigger);


        if(gamepad1.a) {
            telemetry.addData("A Button", "pressed");
        }
        else{
            telemetry.addData("A Button", "is not pressed");
        }

        if(gamepad1.b) {
            telemetry.addData("B Button", "pressed");
        }
        else {
            telemetry.addData("B Button", "is not pressed");
        }

        double differenceBetweenSticks;
        differenceBetweenSticks = gamepad1.left_stick_y - gamepad1.right_stick_y;
        telemetry.addData("Difference Between Left Joystick y and Right Joystick Y", differenceBetweenSticks);

        double sumOfTriggers;
        sumOfTriggers = -gamepad1.left_trigger + gamepad1.right_trigger;
        telemetry.addData("Sum of left and right triggers", sumOfTriggers);







    }


}

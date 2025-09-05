package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotProgrammingPractice extends OpMode {
   private DcMotor frontleftdrive = null;

   private DcMotor backleftdrive = null;

   private DcMotor frontrightdrive = null;

   private DcMotor backrightdrive = null;

    @Override
    public void init() {

        frontleftdrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backleftdrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontrightdrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backrightdrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontleftdrive.setDirection(DcMotor.Direction.REVERSE);
        backleftdrive.setDirection(DcMotor.Direction.REVERSE);
        frontrightdrive.setDirection(DcMotor.Direction.FORWARD);
        backrightdrive.setDirection(DcMotor.Direction.FORWARD);

    }

    @Override
    public void loop() {

        double max;

        double axial = gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        double frontleftpower = axial + lateral + yaw;
        double frontrightpower = axial - lateral - yaw;
        double backleftpower = axial - lateral + yaw;
        double backrightpower = axial + lateral - yaw;

        max = Math.max(Math.abs(frontleftpower), Math.abs(frontrightpower));
        max = Math.max(max, Math.abs(backleftpower));
        max = Math.max(max, Math.abs(backrightpower));

        if (max > 1.0) {
            frontleftpower /= max;
            frontrightpower /= max;
            backleftpower /= max;
            backrightpower /= max;
        }
    frontleftdrive.setPower(frontleftpower);
        frontrightdrive.setPower(frontrightpower);
        backleftdrive.setPower(backleftpower);
        backrightdrive.setPower(backrightpower);


    }

    }




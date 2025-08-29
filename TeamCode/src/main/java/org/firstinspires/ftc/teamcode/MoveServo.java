package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="servo0")
public class MoveServo extends LinearOpMode {
    Servo servo;
    Servo servo2;


    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "servo");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        double position = 0;
        double diff = 0.0005;

        blackboard.put("TestNum", 2);

        waitForStart();
        while (opModeIsActive()){
            servo.setPosition(position);
            servo2.setPosition(position);
            position += diff;

            if (position >= 0.9995) {
                diff = -0.0005;
            } else if (position <= 0.0005) {
                diff = 0.0005;
            }
        }
    }
}

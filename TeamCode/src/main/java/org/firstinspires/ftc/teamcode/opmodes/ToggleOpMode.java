package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBoard;

@TeleOp()
public class ToggleOpMode extends OpMode {
    ProgrammingBoard board = new ProgrammingBoard(); // class member of type ProgrammingBoard1 named board and set it equal to a new instance of ProgrammingBoard1
    boolean aAlreadyPressed;
    boolean motorOn;

    @Override
    public void init() {
        board.init(hardwareMap); //calls init of board object (hardwareMap is how we see how robot is configured)
    }

    @Override
    public void loop() {
       if(gamepad1.a && !aAlreadyPressed) { // if gamepad1.a is true (pressed) AND aAlreadyPressed is false (not pressed) then...
           motorOn = !motorOn;
           telemetry.addData("Motor", motorOn);
           if (motorOn) {
               board.setMotorSpeed(0.5);
           } else {
               board.setMotorSpeed(0.0);
           }
       }
       aAlreadyPressed = gamepad1.a;
    }
}

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBoard;

@TeleOp()
public class MotorOpMode2 extends OpMode {
    ProgrammingBoard board = new ProgrammingBoard(); // class member of type ProgrammingBoard1 named board and set it equal to a new instance of ProgrammingBoard1
    @Override
    public void init() {
        board.init(hardwareMap); //calls init of board object (hardwareMap is how we see how robot is configured)
    }

    @Override
    public void loop() {
        board.setMotorSpeed(0.5);
        telemetry.addData("Motor rotations", board.getMotorRotations());
    }

}

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBoard;

@TeleOp()
public class DistanceColorOpMode extends OpMode {
    ProgrammingBoard board = new ProgrammingBoard(); // class member of type ProgrammingBoard1 named board and set it equal to a new instance of ProgrammingBoard1
    @Override
    public void init() {
        board.init(hardwareMap); //calls init of board object (hardwareMap is how we see how robot is configured)
    }

    @Override
    public void loop() {
        telemetry.addData("Amount red", board.getAmountRed());
        telemetry.addData("Distance (CM)", board.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance (IN)", board.getDistance(DistanceUnit.INCH));

    }

}

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBoard;

@Autonomous()
public class AutoState1 extends OpMode {
    ProgrammingBoard board = new ProgrammingBoard();
    int state;
    @Override
    public void init() {
        board.init(hardwareMap);
    }

    @Override
    public void start() {
        state = 0;
    }

    @Override
    public void loop() {
        telemetry.addData("State", state);
        if (state == 0) {
            board.setServoPosition(0.5);
            if (board.isTouchSensorPressed()) {
                state = 1;
            }
        else if (state == 1) {
            board.setServoPosition(0.0);
            if (!board.isTouchSensorPressed()) {
                state = 2;
            }
        else if (state == 2) {
            board.setServoPosition(1.0);
            board.setMotorSpeed(0.5);
            if (board.getPotAngle() > 90) {
                state = 3;
            }
        else if (state == 3) {
            board.setMotorSpeed(0.0);
            state = 4;
        } else {
            telemetry.addData("Auto", "Finished");
        }
    }
}
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBoard;

@Autonomous()
public class AutoState3 extends OpMode {
    ProgrammingBoard board = new ProgrammingBoard();
    String state = "START";

    @Override
    public void init() {
        board.init(hardwareMap);
    }

    @Override
    public void start() {
        state = "START";
    }

    @Override
    public void loop() {
       telemetry.addData("State", state);
       switch (state) {
           case "START":
               board.setServoPosition(0.5);
               if (board.isTouchSensorPressed()) {
                   state = "WAIT FOR SENSOR RELEASE";
               }
               break;
           case "WAIT FOR SENSOR RELEASE":
               board.setServoPosition(0.0);
               if (!board.isTouchSensorPressed()) {
                   state = "WAIT FOR POT TURN";
               }
               break;
           case "WAIT FOR POT TURN":
               board.setServoPosition(1.0);
               board.setMotorSpeed(0.5);
               if (board.getPotAngle() > 90) {
                   state = "STOP";
               }
               break;
           case "STOP":
               board.setMotorSpeed(0.0);
               state = "DONE";
               break;
           default:
               telemetry.addData("Auto", "Finished");
       }
    }
}
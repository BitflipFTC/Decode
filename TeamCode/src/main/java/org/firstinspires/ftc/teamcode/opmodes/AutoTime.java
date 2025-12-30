package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBoard;

@Autonomous()
public class AutoTime extends OpMode {
    enum State {
        START,
        SECOND_STEP,
        DONE
    }

    ProgrammingBoard board = new ProgrammingBoard();
    State state = State.START;
    double lastTime;

    @Override
    public void init() {
        board.init(hardwareMap);
    }

    @Override
    public void start() {
        state = State.START;
        resetStartTime();
        lastTime = getRuntime();
    }

    @Override
    public void loop() {
       telemetry.addData("State", state);
       telemetry.addData("Runtime", getRuntime());
       telemetry.addData("Time in state", getRuntime() - lastTime);
       switch (state) {
           case START:
               if (getRuntime() >= 3.0) {
                   state = State.SECOND_STEP;
                   lastTime = getRuntime();
               }
               break;
           case SECOND_STEP:
               if (getRuntime() >= lastTime + 3.0) {
                   state = State.DONE;
                   lastTime = getRuntime();
               }
               break;
           default:
               telemetry.addData("Auto", "Finished");
       }
    }
}
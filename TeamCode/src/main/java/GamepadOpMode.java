import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class GamepadOpMode extends OpMode {
    @Override
    public void init() {
    }

    @Override
    public void loop() {
        telemetry.addData("Left stick x", gamepad1.left_stick_x);
        telemetry.addData("Left stick y", gamepad1.left_stick_y);
        telemetry.addData("A button", gamepad1.a);
        telemetry.addData("Right stick x", gamepad1.right_stick_x);
        telemetry.addData("Right stick y", gamepad1.right_stick_y);
        telemetry.addData("B button", gamepad1.b);
        telemetry.addData("Left trigger", gamepad1.left_trigger);
        telemetry.addData("Right trigger", gamepad1.right_trigger);
        telemetry.addData("B Button", "pressed");

        if(gamepad1.b) {
            telemetry.addData("A Button", "pressed");
        }
        else{
            telemetry.addData("A Button", "is not pressed");
        }

        double differenceBetweenSticks;
        differenceBetweenSticks = gamepad1.left_stick_y - gamepad1.right_stick_y;
        telemetry.addData("Difference Between Left Joystick y and Right Joystick Y", differenceBetweenSticks);

        double sumOfTriggers;
        sumOfTriggers = gamepad1.left_trigger + gamepad1.right_trigger;
        telemetry.addData("Sum of left and right triggers", sumOfTriggers);



    }


}

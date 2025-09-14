import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp
public class IfElseOpMode extends OpMode {
    @Override
    public void init() {
    }

    @Override
    public void loop() {
        if (gamepad1.left_stick_y < -0.5) {
            telemetry.addData("Left stick", "is negative and large");
        } else if (gamepad1.left_stick_y < 0) {
            telemetry.addData("Left stick", "is negative and small");
        } else if (gamepad1.left_stick_y < 0.5) {
            telemetry.addData("Left stick", "is positive and small");
        } else {
            telemetry.addData("Left stick", "is positive and large");
        }
        telemetry.addData("Left stick y", gamepad1.left_stick_y);

        while(angle > 180) {
            angle -= 360; // x = x - 360?
        }
        while(angle < -180) {
            angle += 360; // x = x + 360?
        }
        for(int i = 0; i < 4; i++) {
            //this code will happen four times = i starts as 0 and gets one added at the end of each time
        }
    }
}

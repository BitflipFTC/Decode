package org.firstinspires.ftc.teamcode

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.util.CombinedTelemetry

@Configurable
@TeleOp(name = "drive")
class `2wheelDrive` : LinearOpMode() {
    val front by lazy { hardwareMap["front"] as DcMotorEx }
    val back  by lazy { hardwareMap["back"] as DcMotorEx }

    @Throws(InterruptedException::class)
    override fun runOpMode() {

        val combinedTelemetry = CombinedTelemetry(this);
        combinedTelemetry.addLine("Initialized.")
        combinedTelemetry.update()

        var slowspeed : Boolean = true

        back.direction = back.direction.inverted()

        waitForStart()
        while (opModeIsActive()) {
            var motorPower : Double = 1.toDouble()
            if (slowspeed) {
                motorPower *= 0.25;
            }

            if (gamepad1.xWasPressed()) slowspeed = !slowspeed
            var vertical : Double = gamepad1.left_stick_y.toDouble() * motorPower
            var horizontal : Double = -gamepad1.left_stick_x.toDouble() * motorPower

            combinedTelemetry.addData("Slowmode?", slowspeed)

            front.power = vertical + horizontal
            back.power = vertical - horizontal

            combinedTelemetry.update()
        }
    }
}
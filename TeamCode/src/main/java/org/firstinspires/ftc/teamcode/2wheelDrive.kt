package org.firstinspires.ftc.teamcode

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Configurable
@TeleOp(name = "drive")
class `2wheelDrive` : LinearOpMode() {
    private val front by lazy { hardwareMap["front"] as DcMotorEx }
    private val back  by lazy { hardwareMap["back"] as DcMotorEx }

    @Throws(InterruptedException::class)
    override fun runOpMode() {

        val telemetryM = PanelsTelemetry.telemetry;
        telemetryM.debug("Initialized.")
        telemetryM.update()

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

            telemetryM.addData("Slowmode?", slowspeed)

            front.power = vertical + horizontal
            back.power = vertical - horizontal

            telemetryM.update(telemetry)
        }
    }
}
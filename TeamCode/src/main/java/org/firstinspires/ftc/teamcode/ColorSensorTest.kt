package org.firstinspires.ftc.teamcode

import android.graphics.Color
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ColorSensor

@TeleOp(name = "Color sensor test")
class ColorSensorTest : LinearOpMode() {
    val colorSensor by lazy { hardwareMap["color"] as ColorSensor }
    override fun runOpMode() {
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
        waitForStart()
        while (opModeIsActive()) {
            telemetry.addData("Red", colorSensor.red())
            telemetry.addData("Green", colorSensor.green())
            telemetry.addData("Blue", colorSensor.blue())
            telemetry.addData("Alpha", colorSensor.alpha())
            telemetry.addData("ARGB", colorSensor.argb())

            telemetry.addData("r", Color.red(colorSensor.argb()))
            telemetry.addData("g", Color.blue(colorSensor.argb()))
            telemetry.addData("b", Color.green(colorSensor.argb()))

            telemetry.update()
        }
    }
}

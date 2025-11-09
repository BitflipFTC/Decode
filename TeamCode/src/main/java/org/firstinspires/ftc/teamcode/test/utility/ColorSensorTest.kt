package org.firstinspires.ftc.teamcode.test.utility

import android.graphics.Color
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DistanceSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.subsystems.Intake

@TeleOp(name = "Test: Color Sensor", group = "Test")
class ColorSensorTest : LinearOpMode() {
    val colorSensor by lazy { hardwareMap["colorSensor"] as ColorRangeSensor }
    override fun runOpMode() {
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
        waitForStart()
        while (opModeIsActive()) {
            telemetry.addData("Red", colorSensor.red())
            telemetry.addData("Green", colorSensor.green())
            telemetry.addData("Blue", colorSensor.blue())
            telemetry.addData("Alpha", colorSensor.alpha())
            val colors = colorSensor.normalizedColors
            telemetry.addData("i2c address", colorSensor.i2cAddress)
            telemetry.addData("Distance", "%05.2fcm", colorSensor.getDistance(DistanceUnit.CM))

            telemetry.addLine()
            telemetry.addData("r", colors.red / colors.alpha)
            telemetry.addData("g", colors.green / colors.alpha)
            telemetry.addData("b", Color.green(colorSensor.argb()))

            telemetry.update()
        }
    }
}
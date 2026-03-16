package org.firstinspires.ftc.teamcode.test.tuning

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Spindexer
import org.firstinspires.ftc.teamcode.util.TelemetryImplUpstreamSubmission

@TeleOp(name = "Test: Color Sensor", group = "Test")
class ColorSensorTest : LinearOpMode() {
    override fun runOpMode() {
        val colorSensor = ColorSensor()

        colorSensor.initialize()
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, TelemetryImplUpstreamSubmission(this))
        waitForStart()
        while (opModeIsActive()) {
            colorSensor.periodic()

            telemetry.addData("Distance", "%05.2fcm", colorSensor.distance)
            telemetry.addData("Hue","%07.4f", colorSensor.hsv.h)
            telemetry.addData("Saturation","%07.4f", colorSensor.hsv.s)
            telemetry.addData("Value","%07.4f", colorSensor.hsv.v)
            telemetry.update()
        }
    }
}
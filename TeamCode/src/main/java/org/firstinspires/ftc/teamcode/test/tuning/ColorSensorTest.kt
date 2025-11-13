package org.firstinspires.ftc.teamcode.test.tuning

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.ArtifactColorSensor

@TeleOp(name = "Test: Color Sensor", group = "Test")
class ColorSensorTest : LinearOpMode() {
    override fun runOpMode() {
        val colorSensor = ArtifactColorSensor()
        colorSensor.initialize()
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
        waitForStart()
        while (opModeIsActive()) {
            colorSensor.periodic()

            telemetry.update()
        }
    }
}
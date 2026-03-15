package org.firstinspires.ftc.teamcode.test.tuning

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Spindexer
import org.firstinspires.ftc.teamcode.util.TelemetryImplUpstreamSubmission

@TeleOp(name = "Test: Color Sensor (NEW!!)", group = "Test")
class NewColorSensorTest : LinearOpMode() {
    override fun runOpMode() {
        val colorSensor = ColorSensor()
        colorSensor.old = false

        colorSensor.initialize()
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, TelemetryImplUpstreamSubmission(this))
        waitForStart()
        while (opModeIsActive()) {
            colorSensor.periodic()

//
            telemetry.update()
        }
    }
}
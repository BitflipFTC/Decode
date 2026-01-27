package org.firstinspires.ftc.teamcode.test.tuning

import android.util.Size
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Spindexer

@TeleOp(name = "Test: Color Sensor", group = "Test")
class ColorSensorTest : LinearOpMode() {
    override fun runOpMode() {
        val colorSensor = ColorSensor()
        val intake = Intake()
        val spindexer = Spindexer()
//        val colorProcessor = ColorSensorPipeline()
//        val visionPortal = VisionPortal.Builder()
//            .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
//            .setCameraResolution(Size(160, 120))
//            .setShowStatsOverlay(true)
//            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//            .enableLiveView(    true)
//            .addProcessor(colorProcessor)
//            .setAutoStopLiveView(true)
//            .build()
//
//
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
        waitForStart()
        while (opModeIsActive()) {
            colorSensor.periodic()
            spindexer.periodic()
            intake.periodic()

            if (gamepad1.leftBumperWasPressed()) {
                spindexer.toNextOuttakePosition()
            }

            if (gamepad1.rightBumperWasPressed()) {
                spindexer.toNextIntakePosition()
            }

            if (gamepad1.squareWasPressed()) intake.toggle()

//            telemetry.addData("Artifact", colorProcessor.artifact)
//            telemetry.addData("H", "%.1f", colorProcessor.averages.hue)
//            telemetry.addData("S", "%.1f", colorProcessor.averages.saturation)
//            telemetry.addData("V", "%.1f", colorProcessor.averages.value)
//            telemetry.addData("FPS", visionPortal.fps)
//
            telemetry.update()
        }
    }
}
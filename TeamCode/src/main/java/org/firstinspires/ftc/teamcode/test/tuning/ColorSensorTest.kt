package org.firstinspires.ftc.teamcode.test.tuning

import android.util.Size
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.util.ColorSensorPipeline
import org.firstinspires.ftc.vision.VisionPortal

@TeleOp(name = "Test: Color Sensor", group = "Test")
class ColorSensorTest : LinearOpMode() {
    override fun runOpMode() {
//        val colorSensor = ArtifactColorSensor()
        val colorProcessor = ColorSensorPipeline(this.telemetry)
        val visionPortal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .setCameraResolution(Size(160, 120))
            .setShowStatsOverlay(true)
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .enableLiveView(true)
            .addProcessor(colorProcessor)
            .setAutoStopLiveView(true)
            .build()
//        colorSensor.initialize()
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
        waitForStart()
        while (opModeIsActive()) {
//            colorSensor.periodic()

            telemetry.addData("Detected Artifact", colorProcessor.analysis)
            telemetry.addData("FPS", visionPortal.fps)
            telemetry.addData("H", colorProcessor.avgH)
            telemetry.addData("S", colorProcessor.avgS)
            telemetry.addData("V", colorProcessor.avgV)

            telemetry.update()
        }
    }
}
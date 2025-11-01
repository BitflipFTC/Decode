package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.utils.LoopTimer
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.hardware.OV9281
import org.firstinspires.ftc.teamcode.hardware.Turret
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection

@TeleOp(name = "Test: Turret", group = "Test")
class TurretTest : LinearOpMode() {
    var currentTagBearing: Double = 0.0
    var targetTagBearing = 0.0
    override fun runOpMode() {
        telemetry = JoinedTelemetry(
            PanelsTelemetry.ftcTelemetry,
            telemetry,
            FtcDashboard.getInstance().telemetry
        )

        val turret = Turret(this)
        val camera = OV9281(this, 4, 6)

        // bulk caching
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        allHubs.forEach { hub -> hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        telemetry.addLine("Initialized. Press play")
        telemetry.update()

        waitForStart()
        val timer = LoopTimer()
        timer.start()


        while (opModeIsActive()) {
            // more bulk caching
            allHubs.forEach { hub -> hub.clearBulkCache() }

            // atag stuff

            val currentDetections: ArrayList<AprilTagDetection> =
                camera.aprilTag.detections

            val targetDetections = currentDetections.filter { it.metadata.name.contains("RedTarget") }

            currentTagBearing = when {
                targetDetections.isEmpty() -> targetTagBearing
                else                        -> -targetDetections[0].ftcPose.bearing
            }

            turret.bearing = currentTagBearing
            turret.periodic()

            telemetry.addData("current tag bearing", currentTagBearing)
            telemetry.addData(" target tag bearing", targetTagBearing)
            telemetry.addData("turret power", turret.getPower())
            telemetry.addData("At Setpoint?", turret.atSetPoint())
            telemetry.addData("Loop rate", timer.ms)


            telemetry.update()
        }
    }
}

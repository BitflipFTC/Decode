package org.firstinspires.ftc.teamcode.test.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.utils.LoopTimer
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.OV9281
import org.firstinspires.ftc.teamcode.subsystems.Turret
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection

@TeleOp(name = "Test: Turret", group = "Test")
class TurretTest : LinearOpMode() {
    override fun runOpMode() {
        telemetry = JoinedTelemetry(
            PanelsTelemetry.ftcTelemetry,
            telemetry,
            FtcDashboard.getInstance().telemetry
        )

        val turret = Turret()
        val camera = OV9281()
        val subsystems = setOf(turret,camera)
        subsystems.forEach { it.initialize() }
        camera.targetID = 24

        // bulk caching
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        allHubs.forEach { hub -> hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        telemetry.addLine("Initialized. Press play")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            // more bulk caching
            allHubs.forEach { hub -> hub.clearBulkCache() }

            subsystems.forEach { it.periodic() }

            turret.bearing = camera.adjustedTagTargetBearing

            telemetry.addData("turret power", turret.getPower())
            telemetry.addData("At Setpoint?", turret.atSetPoint())

            telemetry.update()
        }
    }
}

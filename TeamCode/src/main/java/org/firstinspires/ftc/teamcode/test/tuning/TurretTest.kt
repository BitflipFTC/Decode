package org.firstinspires.ftc.teamcode.test.tuning

import com.bylazar.field.Style
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing
import org.firstinspires.ftc.teamcode.subsystems.OV9281
import org.firstinspires.ftc.teamcode.subsystems.Turret
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.TelemetryImplUpstreamSubmission

@TeleOp(name = "Test: Turret", group = "Test")
class TurretTest : LinearOpMode() {
    override fun runOpMode() {
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, TelemetryImplUpstreamSubmission(this))

        val turret = Turret()
        val subsystems = setOf(turret)
        subsystems.forEach { it.initialize() }
        turret.selectedAlliance = Alliance.RED

        // bulk caching
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        allHubs.forEach { hub -> hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        telemetry.addLine("Initialized. Press play")
        telemetry.update()

        Drawing.init()
        waitForStart()

        while (opModeIsActive()) {
            // more bulk caching
            allHubs.forEach { hub -> hub.clearBulkCache() }
            turret.robotPose = Pose(72.0, 72.0, 90.0)

            Drawing.drawRobot(turret.robotPose)
            Drawing.drawRobot(turret.goalPose)
            Drawing.sendPacket()

            subsystems.forEach { it.periodic() }

            telemetry.update()
        }
    }
}

package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.hardware.OV9281
import org.firstinspires.ftc.teamcode.util.PIDController
import org.firstinspires.ftc.teamcode.hardware.Turret
import org.firstinspires.ftc.teamcode.util.TurretTestPID.kD
import org.firstinspires.ftc.teamcode.util.TurretTestPID.kV
import org.firstinspires.ftc.teamcode.util.TurretTestPID.kI
import org.firstinspires.ftc.teamcode.util.TurretTestPID.kP
import org.firstinspires.ftc.teamcode.util.TurretTestPID.maxIntegral
import org.firstinspires.ftc.teamcode.util.TurretTestPID.minIntegral
import org.firstinspires.ftc.teamcode.util.TurretTestPID.setPointTolerance
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.util.TurretTestPID.exposure
import org.firstinspires.ftc.teamcode.util.TurretTestPID.kS
import org.firstinspires.ftc.teamcode.util.TurretTestPID.targetTagPos
import org.firstinspires.ftc.teamcode.util.TurretTestPID.tuneKs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign

@TeleOp(name = "Test: Turret", group = "Test")
class TurretTest : LinearOpMode() {
    var currentTagPos: Double = 320.0
    override fun runOpMode() {
        telemetry = JoinedTelemetry(
            PanelsTelemetry.ftcTelemetry,
            telemetry,
            FtcDashboard.getInstance().telemetry
        )

        val turret = Turret(this)
        val camera = OV9281(this, 4, 6)

        val controller = PIDController(kP, kI, kD, kV, kS, maxIntegral, minIntegral)

        // bulk caching
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        allHubs.forEach { hub -> hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        telemetry.addLine("Initialized. Press play")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            // more bulk caching
            allHubs.forEach { hub -> hub.clearBulkCache() }

            camera.setExposure(exposure)

            // atag stuff

            val currentDetections: ArrayList<AprilTagDetection> =
                camera.aprilTag.detections

            currentTagPos = when {
                currentDetections.isEmpty() -> targetTagPos
                else                        -> currentDetections[0].center.x
            }

            controller.setCoeffs(kP, kI, kD, kV, kS)
            controller.setPointTolerance = setPointTolerance
            
            var pidOutput = controller.calculate(currentTagPos, targetTagPos)

            if (!controller.atSetPoint()) {
                pidOutput += sign(pidOutput) * kS
            }

            turret.setPower(pidOutput)

            telemetry.addData("current tag pos", currentTagPos)
            telemetry.addData(" target tag pos", targetTagPos)
            telemetry.addData("turret power", pidOutput)
            telemetry.addData("At Setpoint?", controller.atSetPoint())

            telemetry.update()
        }
    }
}

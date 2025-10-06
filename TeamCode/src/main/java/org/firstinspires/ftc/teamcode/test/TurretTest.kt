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
import org.firstinspires.ftc.teamcode.util.TurretTestPID.kF
import org.firstinspires.ftc.teamcode.util.TurretTestPID.kI
import org.firstinspires.ftc.teamcode.util.TurretTestPID.kP
import org.firstinspires.ftc.teamcode.util.TurretTestPID.maxIntegral
import org.firstinspires.ftc.teamcode.util.TurretTestPID.minIntegral
import org.firstinspires.ftc.teamcode.util.TurretTestPID.setPointTolerance
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sqrt
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.util.TurretTestPID.exposure
import org.firstinspires.ftc.teamcode.util.TurretTestPID.kS
import org.firstinspires.ftc.teamcode.util.TurretTestPID.tuneKs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign

@TeleOp(name = "Test: Turret", group = "Test")
class TurretTest : LinearOpMode() {
    var targetTagPos : Double = 320.0
    var currentTagPos : Double = 320.0
    override fun runOpMode() {
        telemetry = JoinedTelemetry(
            PanelsTelemetry.ftcTelemetry,
            telemetry,
            FtcDashboard.getInstance().telemetry
        )

        val turret = Turret(hardwareMap)
        val hood by lazy { hardwareMap["hood"] as Servo }
        var hoodPos : Double = 0.0
        val camera = OV9281(this,4,6)

        val controller = PIDController(kP, kI, kD, kF, maxIntegral, minIntegral);


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

            val currentDetections: List<AprilTagDetection> =
                camera.aprilTag.detections

            if (currentDetections.isEmpty()) currentTagPos = 320.0
            if (!currentDetections.isEmpty() && currentDetections.get(0).metadata.name.contains("Obelisk"))
                currentTagPos = currentDetections.get(0).center.x

            hood.position = hoodPos
            hoodPos += (gamepad1.right_stick_y * 0.005)
            hoodPos = max((0).toDouble(), min(hoodPos, 0.45))

            controller.setCoeffs(kP, kI, kD, kF)
            controller.setPointTolerance = setPointTolerance
            var pidError = controller.calculate(currentTagPos, targetTagPos)
            if (!controller.atSetPoint()) {pidError += sign(pidError) * kS}

            if (tuneKs) turret.setPower(kS) else turret.setPower(pidError)

            telemetry.addData("current tag pos", currentTagPos)
            telemetry.addData(" target tag pos", targetTagPos)
            telemetry.addData("turret power", pidError * 300)
            telemetry.addData("At Setpoint?",controller.atSetPoint())
            telemetry.addData("Hood pos", hoodPos)

            telemetry.update()
        }
    }
}

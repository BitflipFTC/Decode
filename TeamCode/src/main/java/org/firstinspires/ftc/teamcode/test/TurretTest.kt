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
    var currentTagPos : Double = 320.0
    override fun runOpMode() {
        telemetry = JoinedTelemetry(
            PanelsTelemetry.ftcTelemetry,
            telemetry,
            FtcDashboard.getInstance().telemetry
        )

        val turret = Turret(hardwareMap)
        val hood by lazy { hardwareMap["hood"] as Servo }
        var hoodPos = 0.0
        val camera = OV9281(this,4,6)

        val controller = PIDController(kP, kI, kD, kV, kS,maxIntegral, minIntegral);

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

            if (currentDetections.isEmpty()) currentTagPos = targetTagPos
            if (!currentDetections.isEmpty() && currentDetections[0].metadata.name.contains("Obelisk"))
                currentTagPos = currentDetections[0].center.x

            hood.position = hoodPos
            hoodPos += (gamepad1.right_stick_y * 0.005)
            hoodPos = max((0).toDouble(), min(hoodPos, 0.45))

            controller.setCoeffs(kP, kI, kD, kV,kS)
            controller.setPointTolerance = setPointTolerance
            var pidError = controller.calculate(currentTagPos, targetTagPos)
            if (!controller.atSetPoint()) {pidError += sign(pidError) * kS}

            if (tuneKs) {if (gamepad1.right_bumper) {turret.setPower(kS)}else{turret.setPower(0.0)}} else {turret.setPower(pidError)}

            telemetry.addData("current tag pos", currentTagPos)
            telemetry.addData(" target tag pos", targetTagPos)
            telemetry.addData("turret power", pidError * 300)
            telemetry.addData("At Setpoint?",controller.atSetPoint())
            telemetry.addData("Hood pos", hoodPos)

            telemetry.update()
        }
    }
}

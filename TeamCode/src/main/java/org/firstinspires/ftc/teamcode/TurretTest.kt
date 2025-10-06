package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.util.OV9281
import org.firstinspires.ftc.teamcode.util.PIDController
import org.firstinspires.ftc.teamcode.util.Turret
import org.firstinspires.ftc.teamcode.util.TurretAtagFollow.kD
import org.firstinspires.ftc.teamcode.util.TurretAtagFollow.kF
import org.firstinspires.ftc.teamcode.util.TurretAtagFollow.kI
import org.firstinspires.ftc.teamcode.util.TurretAtagFollow.kP
import org.firstinspires.ftc.teamcode.util.TurretAtagFollow.maxIntegral
import org.firstinspires.ftc.teamcode.util.TurretAtagFollow.minIntegral
import org.firstinspires.ftc.teamcode.util.TurretAtagFollow.setPointTolerance
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sqrt
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.util.TurretAtagFollow.exposure
import org.firstinspires.ftc.teamcode.util.TurretAtagFollow.kS
import org.firstinspires.ftc.teamcode.util.TurretAtagFollow.tuneKs
import org.firstinspires.ftc.teamcode.util.toInt
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign

@TeleOp(name = "Turret Test")
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

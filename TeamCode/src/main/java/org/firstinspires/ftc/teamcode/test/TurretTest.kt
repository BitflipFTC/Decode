package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
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
        val camera = OV9281(this)

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

            // atag stuff

            val currentDetections: List<AprilTagDetection> =
                camera.aprilTag.detections

//            if (currentDetections.isEmpty()) currentTagPos = {320.0
            if (!currentDetections.isEmpty() && currentDetections.get(0).metadata.name.contains("Obelisk"))
                currentTagPos = currentDetections.get(0).center.x


            controller.setCoeffs(kP, kI, kD, kF)
            controller.setPointTolerance = setPointTolerance
            val pidError = controller.calculate(currentTagPos, targetTagPos)

            if (abs(pidError) > 0.01) {
                turret.setPower(pidError);
            } else {
                turret.setPower(0.0);
            }

            telemetry.addData("current tag pos", currentTagPos)
            telemetry.addData(" target tag pos", targetTagPos)
            telemetry.addData("At Setpoint?",controller.atSetPoint())

            telemetry.update()
        }
    }
}

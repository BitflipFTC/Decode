package org.firstinspires.ftc.teamcode.test

import android.util.Size
import com.acmerobotics.dashboard.FtcDashboard
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.hardware.Spindexer
import org.firstinspires.ftc.vision.VisionPortal

@TeleOp(name = "Test: Intake", group = "Test")
class IntakeTest1 : LinearOpMode() {
    val intake by lazy { hardwareMap["intake"] as DcMotorEx }
    val spindexer = Spindexer(hardwareMap)


    override fun runOpMode() {
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, FtcDashboard.getInstance().telemetry, telemetry)
        intake.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        intake.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        intake.direction = DcMotorSimple.Direction.FORWARD
        var camera : WebcamName? = null

        try {
            camera = hardwareMap["Webcam 1"] as WebcamName
        } catch (e: IllegalArgumentException) {
            telemetry.addData("Webcam", "null")
            telemetry.addData("Err", e.toString())
            telemetry.update()
        }

        val visionPortal : VisionPortal
        
        if (camera != null) {
            visionPortal = VisionPortal.Builder()
                .setCamera(hardwareMap["Webcam 1"] as WebcamName)
                .setCameraResolution(Size(320, 240))
                .setShowStatsOverlay(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build()
        }

        waitForStart()

        while(opModeIsActive()) {
            spindexer.update()
            telemetry.addData("Current spindexer position", spindexer.getAngle())
            telemetry.addData("Target spindexer position", spindexer.targetAngle)
            telemetry.addData("Name spindexer position", spindexer.position.name)

            if (gamepad1.crossWasPressed()) {
                spindexer.setPosition(Spindexer.Positions.INTAKE_ZERO)
            }

            if (gamepad1.squareWasPressed()) {
                spindexer.setPosition(Spindexer.Positions.INTAKE_TWO)
            }

            if (gamepad1.triangleWasPressed()) {
                spindexer.setPosition(Spindexer.Positions.OUTTAKE_ONE)
            }

            if (gamepad1.circleWasPressed()) {
                spindexer.toNextPosition()
                gamepad1.rumble(500)
            }

            intake.power = gamepad1.right_trigger.toDouble() - gamepad1.left_trigger.toDouble()
            telemetry.addData("Intake Power", intake.power)
            telemetry.update()
        }

    }
}
package org.firstinspires.ftc.teamcode.test.tuning

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.subsystems.OV9281
import org.firstinspires.ftc.teamcode.util.BetterLoopTimeComponent

@TeleOp(name = "Test: Camera", group = "Test")
class RelocalizationTest: LinearOpMode() {
    override fun runOpMode() {
        val camera = OV9281()
        val dt = Drivetrain()
        dt.initialize()
        camera.initialize()
        Drawing.init()
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
        waitForStart()
        BetterLoopTimeComponent.preStartButtonPressed()
        while (opModeIsActive()) {
            camera.periodic()
            telemetry.run{
                addData("avg pose solve time", camera.aprilTag.perTagAvgPoseSolveTime)
                addLine()
                addLine("Camera Pose adjusted:")
                addData("X", camera.robotPose.x)
                addData("Y", camera.robotPose.y)
                addData("Heading", camera.robotPose.heading)
                addLine()
                addLine("Camera Pose New:")
                addData("New x", camera.newPose.x)
                addData("New y", camera.newPose.y)
                addData("New Heading", camera.newPose.heading)
            }

            Drawing.drawRobot(camera.robotPose)
            Drawing.sendPacket()
            dt.periodic()
            dt.setDrivetrainPowers(dt.calculateDrivetrainPowers(gamepad1.left_stick_x.toDouble(), -gamepad1.left_stick_y.toDouble(), gamepad1.right_stick_x.toDouble()))
            telemetry.update()
            BetterLoopTimeComponent.postUpdate()
        }
    }
}
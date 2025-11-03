package org.firstinspires.ftc.teamcode.opmodes.auto

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.hardware.Drivetrain

@Config
@Autonomous(name = "Line Leave Auto", group = "Autonomous", preselectTeleOp = "Combined TeleOp")
class LineLeaveAuto: LinearOpMode() {
    companion object {
        @JvmField var driveTime: Int = 3000
    }

    enum class Start (val strafe: Double, val forward: Double, val instructions: String) {
        RED_NEAR(1.0, 1.0, "Place back of robot against the wall, about 6in from goal"),
        RED_FAR(0.5, 0.0, "Place back of robot against the wall on outermost corner of far launch line"),
        BLUE_NEAR(-0.5, 0.25, "Place back of robot against the wall, about 6in from goal"),
        BLUE_FAR(-0.5, 0.0, "Place back of robot against the wall on outermost corner of far launch line"),
        NONE(0.0, 0.0, "NO POSITION SELECTED")
    }

    var startingPosition = Start.NONE

    override fun runOpMode() {
        val drivetrain = Drivetrain(this)
        val timer = ElapsedTime()

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML)
        telemetry.addData("Drivetrain", "initialized")
        telemetry.update()

        while (opModeInInit()) {
            telemetry.addData("Drivetrain", "initialized")
            telemetry.addLine("PRESS CIRCLE FOR BLUE ALLIANCE FAR")
            telemetry.addLine("PRESS SQUARE FOR RED ALLIANCE NEAR")
            telemetry.addLine("PRESS TRIANGLE FOR BLUE ALLIANCE NEAR")
            telemetry.addLine("PRESS CROSS FOR RED ALLIANCE FAR")
            telemetry.addLine()
            telemetry.addData("Selected position", "<strong>%s</strong>", startingPosition.name)
            telemetry.addData("<strong>INSTRUCTIONS</strong>", startingPosition.instructions)

            startingPosition = if (gamepad1.crossWasPressed()) {
                Start.RED_FAR
            } else if (gamepad1.triangleWasPressed()) {
                Start.BLUE_NEAR
            } else if (gamepad1.squareWasPressed()) {
                Start.RED_NEAR
            } else if (gamepad1.circleWasPressed()) {
                Start.BLUE_FAR
            } else {
                startingPosition
            }

            telemetry.update()
        }

        waitForStart()
        timer.reset()

        drivetrain.setDrivetrainPowers(drivetrain.calculateDrivetrainPowers(startingPosition.strafe, startingPosition.forward, 0.0))

        // wait for driveTime milliseconds
        while (timer.milliseconds() <= driveTime && opModeIsActive()) {
            telemetry.addData("Moving","...")
            telemetry.addData("forward",startingPosition.forward)
            telemetry.addData("strafe", startingPosition.strafe)
            telemetry.update()

        }

        drivetrain.setDrivetrainPowers(drivetrain.calculateDrivetrainPowers(0.0,0.0,0.0))
    }
}
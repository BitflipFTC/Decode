package org.firstinspires.ftc.teamcode.test.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Spindexer
import org.firstinspires.ftc.teamcode.subsystems.Transfer

@TeleOp(name = "Test: Intake", group = "Test")
class IntakeTest : LinearOpMode() {
    override fun runOpMode() {
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, FtcDashboard.getInstance().telemetry, telemetry)
        val spindexer = Spindexer()
        val intake = Intake()
        val transfer = Transfer()

        waitForStart()

        while(opModeIsActive()) {
            spindexer.periodic()
            telemetry.addData("Current spindexer position", spindexer.currentAngle)
            telemetry.addData("Target spindexer position", spindexer.targetAngle)
            telemetry.addData("Named spindexer position", spindexer.state.name)

            if (gamepad1.dpadUpWasPressed()) {
                transfer.transferArtifact()
            }

            if (gamepad1.dpadDownWasPressed()) {
                transfer.undoTransfer()
            }

            if (gamepad1.crossWasPressed()) {
                spindexer.state = Spindexer.States.INTAKE_ZERO
            }

            if (gamepad1.triangleWasPressed()) {
                spindexer.state = Spindexer.States.OUTTAKE_ZERO
            }

            if (gamepad1.circleWasPressed()) {
                spindexer.toNextPosition()
                gamepad1.rumble(500)
            }

            if (gamepad1.rightBumperWasPressed()) {
                spindexer.toNextOuttakePosition()
            }

            if (gamepad1.leftBumperWasPressed()) {
                spindexer.toNextIntakePosition()
            }

            if (gamepad1.squareWasPressed()) {
                intake.toggle()
            }

            telemetry.addData("Intake Power", intake.power)
            telemetry.update()
        }

    }
}
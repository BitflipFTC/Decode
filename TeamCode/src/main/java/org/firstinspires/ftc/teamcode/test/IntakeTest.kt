package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.hardware.Intake
import org.firstinspires.ftc.teamcode.hardware.Spindexer

@TeleOp(name = "Test: Intake", group = "Test")
class IntakeTest : LinearOpMode() {
    val intake by lazy { hardwareMap["intake"] as DcMotorEx }
    val transferMotor by lazy { hardwareMap["transfer"] as DcMotorEx }


    override fun runOpMode() {
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, FtcDashboard.getInstance().telemetry, telemetry)
        val intake = Intake(this)
        val spindexer = Spindexer(this)

        waitForStart()

        while(opModeIsActive()) {
            spindexer.periodic()
            telemetry.addData("Current spindexer position", spindexer.currentTicks)
            telemetry.addData("Target spindexer position", spindexer.targetTicks)
            telemetry.addData("Name spindexer position", spindexer.state.name)

            transferMotor.power = -gamepad1.right_stick_y.toDouble()
            if (gamepad1.crossWasPressed()) {
                spindexer.setTargetState(Spindexer.States.INTAKE_ZERO)
            }

            if (gamepad1.squareWasPressed()) {
                spindexer.setTargetState(Spindexer.States.INTAKE_TWO)
            }

            if (gamepad1.triangleWasPressed()) {
                spindexer.setTargetState(Spindexer.States.OUTTAKE_ONE)
            }

            if (gamepad1.circleWasPressed()) {
                spindexer.toNextPosition()
                gamepad1.rumble(500)
            }

            intake.power = if (gamepad1.right_trigger >= 0.25) Intake.State.INTAKE else Intake.State.OFF
            telemetry.addData("Intake Power", intake.power)
            telemetry.update()
        }

    }
}
package org.firstinspires.ftc.teamcode.test.tuning

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
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
        val spindexer = Spindexer()
        val intake = Intake()
        val transfer = Transfer()
        val subsystems = setOf(spindexer, intake, transfer)

        subsystems.forEach { it.initialize() }
        waitForStart()

        while(opModeIsActive()) {
            subsystems.forEach { it.periodic() }

            if (gamepad1.dpadUpWasPressed()) {
                intake.power += 0.1
            }

            if (gamepad1.dpadDownWasPressed()) {
                intake.power -= 0.1
            }

            telemetry.addData("Intake Power", intake.power)
            telemetry.update()
        }

    }
}
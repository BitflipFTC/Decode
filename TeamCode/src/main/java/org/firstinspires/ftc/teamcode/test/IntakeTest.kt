package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple

@TeleOp(name = "Test: Intake", group = "Test")
class IntakeTest1 : LinearOpMode() {
    val intake by lazy { hardwareMap["intake"] as DcMotorEx }

    override fun runOpMode() {
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, FtcDashboard.getInstance().telemetry, telemetry)
        intake.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        intake.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        intake.direction = DcMotorSimple.Direction.FORWARD

        waitForStart()

        while(opModeIsActive()) {
            intake.power = gamepad1.right_trigger.toDouble() - gamepad1.left_trigger.toDouble()
            telemetry.addData("Motor Power", intake.power)
            telemetry.update()
        }

    }
}
package org.firstinspires.ftc.teamcode.test.tuning

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.OV9281
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Spindexer
import org.firstinspires.ftc.teamcode.subsystems.Transfer
import org.firstinspires.ftc.teamcode.subsystems.Turret

@TeleOp(name = "Test: Flywheel", group = "Test")
class FlywheelTest : LinearOpMode() {
    override fun runOpMode() {
        val shooter = Shooter()
        val camera = OV9281()
        val transfer = Transfer()
        val intake = Intake()
        val spindexer = Spindexer()
        val turret = Turret()
        val subsystems = setOf(shooter,camera,transfer,intake,spindexer,turret)
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)

        subsystems.forEach { it.initialize() }

        // bulk caching
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        allHubs.forEach { hub -> hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        waitForStart()

        shooter.hoodPosition = 0.5
        camera.targetID = 24
        shooter.periodic()

        while (opModeIsActive()) {
            // more bulk caching
            allHubs.forEach { hub -> hub.clearBulkCache() }

//            // reset the target rpm
            if (gamepad1.dpadDownWasPressed()) {
                shooter.targetFlywheelRPM -= 250
            }
//
            if (gamepad1.dpadUpWasPressed()) {
                shooter.targetFlywheelRPM += 250
            }

            if (gamepad1.dpadRightWasPressed()) {
                shooter.hoodPosition += 0.05
            }

            if (gamepad1.dpadLeftWasPressed()) {
                shooter.hoodPosition -= 0.05
            }

            if (gamepad1.crossWasPressed()) {
                shooter.targetFlywheelRPM = 0.0
            }

            if (gamepad1.circleWasPressed()) {
                shooter.targetFlywheelRPM = 4500.0
            }

            if (gamepad1.triangleWasPressed()) {
                transfer.transferArtifact()
            }

            if (gamepad1.squareWasPressed()) {
                intake.toggle()
            }

            if (gamepad1.leftBumperWasPressed()) {
                spindexer.toNextOuttakePosition()
            } else if (gamepad1.rightBumperWasPressed()) {
                spindexer.toNextIntakePosition()
            }

            if (gamepad1.right_trigger >= 0.15) {
                turret.bearing = camera.currentTagBearing
                turret.turningPower = gamepad1.right_stick_x.toDouble()
            } else {
                turret.bearing = 0.0
                turret.turningPower = 0.0
            }

            subsystems.forEach { it.periodic() }

            telemetry.update()
        }
    }

}
package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.bylazar.battery.PanelsBattery
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.hardware.Shooter

@TeleOp(name = "Test: Flywheel", group = "Test")
class FlywheelTest : LinearOpMode() {
    override fun runOpMode() {
        val shooter = Shooter(this)
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, FtcDashboard.getInstance().telemetry, telemetry)

        // bulk caching
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        allHubs.forEach { hub -> hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        waitForStart()
//        shooter.hoodPosition = 0.2

        while (opModeIsActive()) {
            // more bulk caching
            allHubs.forEach { hub -> hub.clearBulkCache() }

//            // reset the target rpm
//            if (gamepad1.dpadDownWasPressed()) {
//                shooter.targetFlywheelRPM = 3000.0
//            }
//
//            if (gamepad1.dpadUpWasPressed()) {
//                shooter.targetFlywheelRPM = 4500.0
//            }
//
//            if (gamepad1.dpadRightWasPressed()) {
//                shooter.hoodPosition = 0.05
//            }
//
//            if (gamepad1.dpadLeftWasPressed()) {
//                shooter.hoodPosition = 0.5
//            }


            shooter.periodic()

            telemetry.addData("Current RPM", shooter.flywheelRPM)
            telemetry.addData("Target RPM", shooter.targetFlywheelRPM)
            telemetry.addData("Hood position", shooter.hoodPosition)
            telemetry.addData("battery", PanelsBattery.provider.batteryVoltage)
            telemetry.update()
        }
    }

}
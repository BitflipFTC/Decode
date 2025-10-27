package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.hardware.Drivetrain
import org.firstinspires.ftc.teamcode.hardware.Intake
import org.firstinspires.ftc.teamcode.hardware.Shooter
import org.firstinspires.ftc.teamcode.hardware.Spindexer
import org.firstinspires.ftc.teamcode.hardware.Transfer
import org.firstinspires.ftc.teamcode.util.HeadingCorrectPID.targetImuPos

class CombinedTeleOp : LinearOpMode() {
    companion object {
        @JvmField
        var flywheelRPM = 3000.0
        @JvmField
        var hoodPosition = 0.3
    }
    // Drivetrain
    private var fieldCentric : Boolean = false
    private val driveSpeed = 0.8

    override fun runOpMode() {
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry, FtcDashboard.getInstance().telemetry)
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE)
        val drivetrain = Drivetrain(hardwareMap)
        val intake = Intake(hardwareMap)
        val transfer = Transfer(hardwareMap)
        val spindexer = Spindexer(hardwareMap)
        val shooter = Shooter(hardwareMap)

        // bulk caching
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        allHubs.forEach { hub -> hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        waitForStart()

        while (opModeIsActive()) {
            // more bulk caching
            allHubs.forEach { hub -> hub.clearBulkCache() }

            // drrivetrain
            if (gamepad1.bWasPressed()) fieldCentric = !fieldCentric
            if (gamepad1.yWasPressed()) drivetrain.resetYaw()

            drivetrain.driveSpeed = driveSpeed
            drivetrain.fieldCentric = fieldCentric

            drivetrain.setDrivetrainPowers(drivetrain.calculateDrivetrainPowers(
                gamepad1.left_stick_x,
                -gamepad1.left_stick_y,
                gamepad1.right_stick_x))


            // intake
            if (gamepad1.xWasPressed()) intake.toggle()

            // transfer
            if (gamepad1.right_trigger >= 0.25) transfer.transferArtifact(); gamepad1.rumble(500)

            // spindexer
            if (gamepad1.rightBumperWasPressed()) {
                spindexer.toNextOuttakePosition()
                gamepad1.rumble(500)
            }

            if (gamepad1.leftBumperWasPressed()) {
                spindexer.toNextIntakePosition()
                gamepad1.rumble(500)
            }

            // shooter stuff
            shooter.hoodPosition = hoodPosition
            shooter.targetFlywheelRPM = flywheelRPM

            // update all mechanisms
            transfer.update()
            spindexer.update()
            shooter.update()

            telemetry.addData("Drivetrain Powers", drivetrain.currentDrivePowers.toString())
            telemetry.addData("Heading", drivetrain.heading)
            telemetry.addData("Field Centric? ", drivetrain.fieldCentric)
            telemetry.addData("", "-------------------------------------")
            telemetry.addData("Intake Power", intake.power)
            telemetry.addData("", "-------------------------------------")
            telemetry.addData("Transfer Target Ticks", transfer.targetPosition)
            telemetry.addData("Transfer Current Ticks", transfer.currentPosition)
            telemetry.addData("", "-------------------------------------")
            telemetry.addData("Spindexer target position", spindexer.position.name)
            telemetry.addData("Spindexer target angle", spindexer.targetAngle)
            telemetry.addData("Spindexer current angle", spindexer.getAngle())
            telemetry.addData("", "-------------------------------------")
            telemetry.addData("Flywheel target RPM", shooter.targetFlywheelRPM)
            telemetry.addData("FLywheel current RPM", shooter.flywheelRPM)
            telemetry.addData("Hood position", shooter.hoodPosition)

            telemetry.update()
        }
    }
}
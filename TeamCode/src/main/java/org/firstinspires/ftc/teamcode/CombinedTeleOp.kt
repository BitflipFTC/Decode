package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.hardware.Drivetrain
import org.firstinspires.ftc.teamcode.hardware.Intake
import org.firstinspires.ftc.teamcode.hardware.Spindexer
import org.firstinspires.ftc.teamcode.hardware.Transfer
import org.firstinspires.ftc.teamcode.util.HeadingCorrectPID.targetImuPos

class CombinedTeleOp : LinearOpMode() {
    // Drivetrain
    private var fieldCentric : Boolean = false
    private val driveSpeed = 0.8

    override fun runOpMode() {
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry, FtcDashboard.getInstance().telemetry)
        val drivetrain = Drivetrain(hardwareMap)
        val transfer = Transfer(hardwareMap)
        val spindexer = Spindexer(hardwareMap)
        val intake = Intake(hardwareMap)

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
            if (gamepad1.crossWasPressed()) transfer.transferArtifact(); gamepad1.rumble(500)

            // spindexer
            if (gamepad1.rightBumperWasPressed()) {
                spindexer.toNextOuttakePosition()
                gamepad1.rumble(500)
            }

            if (gamepad1.leftBumperWasPressed()) {
                spindexer.toNextIntakePosition()
                gamepad1.rumble(500)
            }



            telemetry.addData("targ. Imu position", targetImuPos)
            telemetry.addData("curr. Imu position", drivetrain.heading)

            telemetry.update()
        }
    }
}
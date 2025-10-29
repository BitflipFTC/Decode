package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import gay.zharel.fateweaver.flight.FlightRecorder
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.hardware.Drivetrain
import org.firstinspires.ftc.teamcode.hardware.Intake
import org.firstinspires.ftc.teamcode.hardware.OV9281
import org.firstinspires.ftc.teamcode.hardware.Shooter
import org.firstinspires.ftc.teamcode.hardware.Spindexer
import org.firstinspires.ftc.teamcode.hardware.Transfer
import org.firstinspires.ftc.teamcode.hardware.Turret
import org.firstinspires.ftc.teamcode.util.toInt
import kotlin.math.pow
import kotlin.math.sqrt

@Config
@TeleOp(name = "Combined TeleOp", group = "TeleOp")
class CombinedTeleOp : LinearOpMode() {
    companion object {
        @JvmField
        var flywheelRPM = 0.0
        @JvmField
        var hoodPosition = 0.2
    }
    // Drivetrain
    private var fieldCentric : Boolean = false
    private val driveSpeed = 0.8
    lateinit var camera: OV9281

    var distanceToGoal = -1.0
    var currentTagPosition = 320.0
    var targetTagPosition = 320.0
    // for dashboard purposes

    override fun runOpMode() {
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry, FtcDashboard.getInstance().telemetry)
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE)
        val drivetrain = Drivetrain(this)
        val intake = Intake(this)
        val transfer = Transfer(this)
        val spindexer = Spindexer(this)
        val shooter = Shooter(this)
        val turret = Turret(this)

        camera = OV9281(this)

        val timer = ElapsedTime()
        val timestampRecorder = FlightRecorder.createChannel("TIMESTAMP", Double::class.java)
        val hoodPositionRecorder = FlightRecorder.createChannel("HOOD POSITION", Double::class.java)
        val flywheelRPMRecorder = FlightRecorder.createChannel("FLYWHEEL RPM", Double::class.java)
        val distanceRecorder = FlightRecorder.createChannel("DISTANCE (IN)", Double::class.java)

        // bulk caching
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        allHubs.forEach { hub -> hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        waitForStart()

        while (opModeIsActive()) {
            // more bulk caching
            allHubs.forEach { hub -> hub.clearBulkCache() }

            // drrivetrain
            if (gamepad1.circleWasPressed()) fieldCentric = !fieldCentric
            if (gamepad1.yWasPressed()) drivetrain.resetYaw()

            drivetrain.driveSpeed = driveSpeed
            drivetrain.fieldCentric = fieldCentric

            drivetrain.setDrivetrainPowers(drivetrain.calculateDrivetrainPowers(
                gamepad1.left_stick_x,
                -gamepad1.left_stick_y,
                gamepad1.right_stick_x))

            // intake
            if (gamepad1.squareWasPressed()) intake.toggle()

            // transfer
//            if (gamepad1.right_trigger >= 0.25) { transfer.transferArtifact(); gamepad1.rumble(500) }
//            if (gamepad1.left_trigger >= 0.25) { transfer.undoTransfer(); gamepad1.rumble(500) }
            transfer.setPower((gamepad1.right_trigger - gamepad1.left_trigger).toDouble());

            // spindexer
//            if (gamepad1.rightBumperWasPressed()) {
//                spindexer.toNextOuttakePosition()
//                gamepad1.rumble(100)
//                spindexer.resetIntegral()
//            }z
//
//            if (gamepad1.leftBumperWasPressed()) {
//                spindexer.toNextIntakePosition()
//                gamepad1.rumble(100)
//                spindexer.resetIntegral()
//            }

            spindexer.setPower(gamepad1.right_bumper.toInt().toDouble() * 0.25 - gamepad1.left_bumper.toInt().toDouble() * 0.25)

            if (gamepad1.dpad_up) {
                flywheelRPM = 4500.0
                hoodPosition = 0.2
            }
            if (gamepad1.dpad_down) {
                flywheelRPM = 0.0
            }
            if (gamepad1.dpad_right) {
                flywheelRPM = 3000.0
                hoodPosition = 0.5
            }

            // shooter stuff
            shooter.hoodPosition = hoodPosition
            shooter.targetFlywheelRPM = flywheelRPM

            // update all mechanisms
//            transfer.periodic()
            shooter.periodic()
            intake.periodic()
            updateCamera()
            turret.periodic(currentTagPosition)

            if (distanceToGoal >= 0.0) {
                gamepad1.setLedColor(255.0, 136.0, 30.0, Gamepad.LED_DURATION_CONTINUOUS)
            } else {
                gamepad1.setLedColor(255.0, 255.0, 255.0, Gamepad.LED_DURATION_CONTINUOUS)
            }

            // for logging optimal flywheel / hood
            if (gamepad1.touchpadWasPressed()) {
                gamepad1.rumble(500)
                gamepad1.setLedColor(255.0, 136.0, 30.0, 500)

                timestampRecorder.put(timer.milliseconds())
                hoodPositionRecorder.put(hoodPosition)
                flywheelRPMRecorder.put(flywheelRPM)
                distanceRecorder.put(distanceToGoal)
            }

            telemetry.addData("Drivetrain Powers", drivetrain.currentDrivePowers.toString())
            telemetry.addData("Heading", drivetrain.heading)
            telemetry.addData("Field Centric? ", drivetrain.fieldCentric)
            telemetry.addData("", "-------------------------------------")
            telemetry.addData("Intake Power", intake.power.value)
            telemetry.addData("Intake State", intake.power)
            telemetry.addLine("-------------------------------------")
            telemetry.addData("Transfer Target Ticks", transfer.targetPosition)
            telemetry.addData("Transfer Current Ticks", transfer.currentPosition)
            telemetry.addLine("-------------------------------------")
            telemetry.addData("Spindexer target position", spindexer.position.name)
            telemetry.addData("Spindexer target ticks", spindexer.getTargetAngle())
            telemetry.addData("Spindexer current ticks", spindexer.getAngle())
            telemetry.addLine("-------------------------------------")
            telemetry.addData("Flywheel target RPM", shooter.targetFlywheelRPM)
            telemetry.addData("FLywheel current RPM", shooter.flywheelRPM)
            telemetry.addData("Hood position", shooter.hoodPosition)
            telemetry.addData("Distance to goal", distanceToGoal)
            telemetry.addLine("-------------------------------------")
            telemetry.addData("April tag current position", currentTagPosition)
            telemetry.addData("April tag target position", targetTagPosition)
            telemetry.addData("Turret power", turret.getPower())

            telemetry.update()
        }
    }

    fun updateCamera() {
        val currentDetections = camera.aprilTag.detections

        if (!currentDetections.isEmpty()) {
            telemetry.addData("Detected april tags", currentDetections.size)

            val detection = currentDetections[0]
            if (detection.metadata != null) {
                telemetry.addData("TAG NAME", detection.metadata.name)

                if (!detection.metadata.name.contains("Obelisk")) {

                    // DISTANCE CALCULATIONS
                    val tagPos = Pose2D(
                        DistanceUnit.INCH,
                        detection.metadata.fieldPosition.get(0).toDouble(),
                        detection.metadata.fieldPosition.get(1).toDouble(),
                        AngleUnit.DEGREES,
                        detection.metadata.fieldOrientation.toOrientation(
                            AxesReference.EXTRINSIC,
                            AxesOrder.XYZ,
                            AngleUnit.DEGREES
                        ).secondAngle.toDouble()
                    ) // given north is 0, clockwise

                    val robotPos = Pose2D(
                        detection.robotPose.getPosition().unit,
                        detection.robotPose.getPosition().x,
                        detection.robotPose.getPosition().y,
                        AngleUnit.DEGREES,
                        detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)
                    )

                    distanceToGoal = sqrt(
                        (tagPos.getX(DistanceUnit.INCH) - robotPos.getX(DistanceUnit.INCH)).pow(
                            2.0
                        ) + (tagPos.getY(DistanceUnit.INCH) - robotPos.getY(DistanceUnit.INCH)).pow(
                            2.0
                        )
                    )
                    // END DISTANCE CALCS

                    currentTagPosition = detection.center.x
                }
            } else {
                telemetry.addData("Current tag", "NO metadata")
            }
        } else { // no detections
            telemetry.addData("Detected april tags", 0)
        }

    }
}
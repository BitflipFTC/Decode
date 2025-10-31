package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.ShootPreloadsAuto.Start
import org.firstinspires.ftc.teamcode.hardware.Drivetrain
import org.firstinspires.ftc.teamcode.hardware.Intake
import org.firstinspires.ftc.teamcode.hardware.OV9281
import org.firstinspires.ftc.teamcode.hardware.Shooter
import org.firstinspires.ftc.teamcode.hardware.Spindexer
import org.firstinspires.ftc.teamcode.hardware.Transfer
import org.firstinspires.ftc.teamcode.hardware.Turret
import org.firstinspires.ftc.teamcode.util.dpadToAxes
import kotlin.math.abs
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
    var rangeDistanceToGoal = -1.0
    var currentTagBearing = 0.0
    val targetTagBearing = 0.0
    // for dashboard purposes

    enum class Alliance (val atagTarget: String) {
        RED("RedTarget"),
        BLUE("BlueTarget"),
        NONE("NONE")
    }

    var alliance: Alliance = Alliance.NONE

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

        while (opModeInInit()) {
            telemetry.addData("Drivetrain", "initialized")
            telemetry.addLine("Press CIRCLE for Red  alliance")
            telemetry.addLine("Press CROSS  for Blue alliance")
            telemetry.addLine()
            telemetry.addData("Selected alliance", "<strong>%s</strong>", alliance.name)
            telemetry.addData("Selected goal", alliance.atagTarget)

            alliance = if (gamepad1.crossWasPressed()) {
                Alliance.BLUE
            } else if (gamepad1.circleWasPressed()) {
                Alliance.RED
            } else {
                alliance
            }

            telemetry.update()
        }

        // bulk caching
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        allHubs.forEach { hub -> hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

//        waitForStart()
        // todo i dont think we need wait for start???

        while (opModeIsActive()) {
            // more bulk caching
            allHubs.forEach { hub -> hub.clearBulkCache() }

            // see https://www.padcrafter.com/?templates=Driver+1%7CDriver+2&plat=1%7C1&col=%23242424%2C%23606A6E%2C%23FFFFFF&leftStick=Drivetrain+translation%7CTurret+Manual+Control&rightStick=Drivetrain+rotation%7CSpindexer+Manual+Control&yButton=Actuate+Transfer&xButton=Toggle+intake&rightTrigger=Reverse+Intake%7CTransfer+forward&rightBumper=Spindexer%3A+Next+Outtake&leftBumper=Spindexer%3A+Next+Intake&leftTrigger=Auto-align+turret+%2B+Auto+adjust+shooter%7CTransfer+backward&bButton=Reset+yaw+%28TO+BE+REMOVED%29&backButton=Toggle+field+centric&dpadLeft=%7CHoodPosition+-+0.05&dpadRight=%7CHoodPosition+%2B+0.05&dpadUp=%7CFlywheel+RPM+%2B+500&dpadDown=%7CFlywheel+RPM+-+500&aButton=Move+transfer+back+down+if+stuck%7C
            // for gamepad layouts

            // drivetrain
            if (gamepad1.touchpadWasPressed()) fieldCentric = !fieldCentric
            if (gamepad1.circleWasPressed()) drivetrain.resetYaw()
            drivetrain.setDrivetrainPowers(drivetrain.calculateDrivetrainPowers(
                gamepad1.left_stick_x,
                -gamepad1.left_stick_y,
                gamepad1.right_stick_x))

            drivetrain.driveSpeed = driveSpeed
            drivetrain.fieldCentric = fieldCentric


            // intake
            if (gamepad1.squareWasPressed()) intake.toggle()

            // while held, reversed
            if (gamepad1.right_trigger >= 0.25) {
                intake.reversed = true
            } else { intake.reversed = false }

            // transfer
            if (gamepad1.triangleWasPressed()) { transfer.transferArtifact(); gamepad1.rumble(500) }
            if (gamepad1.crossWasPressed())    { transfer.undoTransfer(); gamepad1.rumble(500) }
//            transfer.setPower((gamepad1.right_trigger - gamepad1.left_trigger).toDouble());

            // spindexer
            if (gamepad1.rightBumperWasPressed()) {
                spindexer.toNextOuttakePosition()
                gamepad1.rumble(100)
            }

            if (gamepad1.leftBumperWasPressed()) {
                spindexer.toNextIntakePosition()
                gamepad1.rumble(100)
            }

//            spindexer.power = gamepad1.right_bumper.toInt().toDouble() * 0.25 - gamepad1.left_bumper.toInt().toDouble() * 0.25

            // map autoaim behind left trigger
            if (gamepad1.left_trigger >= 0.25) {
                turret.periodic(currentTagBearing)

                // todo uncomment this when the ILUT is populated
//                shooter.calculateTargetState(rangeDistanceToGoal)
            } else {
                turret.setPower(gamepad2.left_stick_x.toDouble())
            }

            // shooter stuff
            // use these to configure ILUT
            // todo move these controls to the else block of autoaim when ilut done
            shooter.hoodPosition = hoodPosition
            shooter.targetFlywheelRPM = flywheelRPM

            // todo move these controls to gamepad2
            val (dpadX, dpadY) = gamepad1.dpadToAxes()
            flywheelRPM += dpadY * 250.0
            hoodPosition += dpadX * 0.05

            // update all mechanisms

            // allow gamepad2 to have manual transfer control
            if (abs(gamepad2.right_trigger - gamepad2.left_trigger) >= 0.15) {
                transfer.setPower((gamepad2.right_trigger - gamepad2.left_trigger).toDouble())
            } else {
                transfer.periodic()
            }

            shooter.periodic()
            intake.periodic()
            updateCamera(alliance.atagTarget)

            if (abs(gamepad2.right_stick_x) >= 0.15) {
                spindexer.power = gamepad2.right_stick_x.toDouble()

                // prevent from periodic() resetting to a preset
                spindexer.targetAngle = spindexer.currentAngle
            } else {
                spindexer.periodic()
            }

            // if we have a detection, green; otherwise, red
            if (distanceToGoal > 0.0) {
                gamepad1.setLedColor(0.0, 255.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS)
            } else {
                gamepad1.setLedColor(255.0, 0.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS)
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
            telemetry.addData("Spindexer target position", spindexer.state.name)
            telemetry.addData("Spindexer target angle", spindexer.targetAngle)
            telemetry.addData("Spindexer current angle", spindexer.currentAngle)
            telemetry.addLine("-------------------------------------")
            telemetry.addData("Flywheel target RPM", shooter.targetFlywheelRPM)
            telemetry.addData("Flywheel current RPM", shooter.flywheelRPM)
            telemetry.addData("Hood position", shooter.hoodPosition)
            telemetry.addData("Distance to goal", distanceToGoal)
            telemetry.addData("RANGE Distance", rangeDistanceToGoal)
            telemetry.addLine("-------------------------------------")
            telemetry.addData("April tag current bearing", currentTagBearing)
            telemetry.addData("April tag target bearing", targetTagBearing)
            telemetry.addData("Turret power", turret.getPower())

            telemetry.update()
        }
    }

    fun updateCamera(target: String) {
        val currentDetections = camera.aprilTag.detections

        if (!currentDetections.isEmpty()) {
            telemetry.addData("Detected april tags", currentDetections.size)

            val detection = currentDetections[0]
            if (detection.metadata != null) {
                telemetry.addData("TAG NAME", detection.metadata.name)

                // if the atag seen is the current target's atag
                if (detection.metadata.name.contains(target)) {
                    rangeDistanceToGoal = detection.ftcPose.range
                    // negative b/c default: left positive, right negative
                    currentTagBearing = -detection.ftcPose.bearing

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
                }
            } else {
                telemetry.addData("Current tag", "NO metadata")
            }
        } else { // no detections
            telemetry.addData("Detected april tags", 0)
        }
    }
}
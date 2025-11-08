package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.utils.LoopTimer
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Spindexer
import org.firstinspires.ftc.teamcode.subsystems.Transfer
import org.firstinspires.ftc.teamcode.subsystems.Turret
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.OV9281
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.dpadToAxes
import kotlin.math.abs

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

    val targetTagBearing = 0.0
    // for dashboard purposes

    var alliance: Alliance = Alliance.NONE

    // shooter logic is as follows
    // when right trigger is held on gp1, if there is an apriltag visible
    // it auto sets rpm and hood.
    // when a preset is pressed on gp2, it auto sets to those UNLESS right
    // trigger is being held down
    // when manual adjustments are made on gp2, it adjusts the targets UNLESS
    // right trigger is held, amd keeps those adjustments UNTIL a preset is assigned
    // OR the right trigger is held

    // spindexer logic is as follows
    // when r1 / l1 is pressed, it snaps to that preset
    // HOWEVER, when right stick is moved on gp2, it sets the new target to that
    // and keeps it there until a new preset is selected or further input
    // is detected

    // turret logic is as follows
    // turret is entirely controlled by gp2 left stick
    // UNLESS gp2 left bumper is held AND apriltag is seen

    // transfer logic
    // uhhhhhhhhhhhhhh
    // triangle shoots and returns to nearest rotation
    // gp2 right + left triggers move it manually
    override fun runOpMode() {
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry, FtcDashboard.getInstance().telemetry)
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML)
        val drivetrain = Drivetrain()
        val intake = Intake()
        val transfer = Transfer()
        val spindexer = Spindexer()
        val shooter = Shooter()
        val turret = Turret()

        camera = OV9281()

        while (opModeInInit()) {
            telemetry.addData("Drivetrain", "initialized")
            telemetry.addLine("Press CIRCLE for Red  alliance")
            telemetry.addLine("Press CROSS  for Blue alliance")
            telemetry.addLine()
            telemetry.addData("Selected alliance", "<strong>%s</strong>", alliance.name)
            telemetry.addData("Selected goal", alliance.aprilTagID)

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

        val timer= ElapsedTime()
        intake.intake()

        camera.targetID = alliance.aprilTagID

        while (opModeIsActive()) {
            // more bulk caching
            allHubs.forEach { hub -> hub.clearBulkCache() }

            // see https://www.padcrafter.com/?templates=Driver+1%7CDriver+2&plat=1%7C1&col=%23242424%2C%23606A6E%2C%23FFFFFF&leftStick=Drivetrain+translation%7CTurret+Manual+Control&rightStick=Drivetrain+rotation%7CSpindexer+Manual+Control&yButton=Actuate+Transfer&xButton=Toggle+intake&rightTrigger=Auto+adjust+shooter%7CTransfer+forward&rightBumper=Spindexer%3A+Next+Outtake&leftBumper=Spindexer%3A+Next+Intake%7CAutoaim+turret&leftTrigger=Reverse+Intake%7CTransfer+backward&bButton=Reset+yaw+%28TO+BE+REMOVED%29%7CLong+shooting+preset&backButton=Toggle+field+centric&dpadLeft=%7CHoodPosition+-+0.05&dpadRight=%7CHoodPosition+%2B+0.05&dpadUp=%7CFlywheel+RPM+%2B+250&dpadDown=%7CFlywheel+RPM+-+250&aButton=%7CClose+shooting+preset
            // for gamepad layouts

            // drivetrain
            if (gamepad1.touchpadWasPressed()) fieldCentric = !fieldCentric
            if (gamepad1.circleWasPressed()) drivetrain.resetYaw()
            drivetrain.setDrivetrainPowers(drivetrain.calculateDrivetrainPowers(
                gamepad1.left_stick_x.toDouble(),
                -gamepad1.left_stick_y.toDouble(),
                gamepad1.right_stick_x.toDouble()))

            drivetrain.driveSpeed = driveSpeed
            drivetrain.fieldCentric = fieldCentric

            camera.periodic()


            // transfer
            if (gamepad1.triangleWasPressed()) { transfer.transferArtifact(); gamepad1.rumble(500) }

            // spindexer
            if (gamepad1.rightBumperWasPressed()) {
                spindexer.toNextOuttakePosition()
                gamepad1.rumble(100)
            }

            if (gamepad1.leftBumperWasPressed()) {
                spindexer.toNextIntakePosition()
                gamepad1.rumble(100)
            }

            // intake
            if (gamepad1.squareWasPressed()) intake.toggle()

            // while held, reversed
            if (gamepad1.crossWasPressed()) {
                intake.reversed = true
            }

            if (gamepad1.crossWasReleased()) {
                intake.reversed = false
            }

            // map autoaim behind left trigger
            if (gamepad1.left_trigger >= 0.25) {
                turret.bearing = camera.currentTagBearing
                turret.periodic()
            } else {
                turret.setPower(gamepad2.left_stick_x.toDouble() * 0.15)
            }

            if (gamepad1.dpadDownWasPressed()) {
                intake.off()
            }

            // map auto adjust behind right trigger
            if (gamepad1.right_trigger >= 0.25) {
                shooter.calculateTargetState(camera.distanceToGoal)
            } else {
                shooter.state = Shooter.ShooterState(hoodPosition, flywheelRPM)
            }

            // shooter stuff
            val (dpadX, dpadY) = gamepad2.dpadToAxes()
            flywheelRPM += dpadY * 250.0
            hoodPosition += dpadX * 0.05

            if (gamepad2.aWasPressed()) {
                shooter.calculateTargetState(Shooter.CLOSE_SHOOTING)
            }
            if (gamepad2.bWasPressed()) {
                shooter.calculateTargetState(Shooter.PEAK_NEAR_LAUNCH_ZONE)
            }

            // update all mechanisms

            // allow gamepad2 to have manual transfer control
            if (abs(gamepad2.right_trigger - gamepad2.left_trigger) >= 0.15) {
                transfer.setPower((gamepad2.right_trigger - gamepad2.left_trigger) * 1.0)
                transfer.targetPosition = transfer.currentPosition
            } else {
                transfer.periodic()
            }

            shooter.periodic()
            intake.periodic()

            if (abs(gamepad2.right_stick_x) >= 0.15) {
                spindexer.power = gamepad2.right_stick_x.toDouble() * 0.75

                // prevent from periodic() resetting to a preset
                spindexer.targetAngle = spindexer.currentAngle
            } else {
                spindexer.periodic()
            }

            // if we have a detection, green; otherwise, red
            if (camera.distanceToGoal > 0.0) {
                gamepad1.setLedColor(0.0, 255.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS)
            } else {
                gamepad1.setLedColor(255.0, 0.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS)
            }

            telemetry.addData("Drivetrain Powers", drivetrain.currentDrivePowers.toString())
            telemetry.addData("Heading", drivetrain.heading)
            telemetry.addData("Field Centric?", drivetrain.fieldCentric)
            telemetry.addLine("-------------------------------------")
            telemetry.addData("Intake Power", intake.power.value)
            telemetry.addData("Intake State", intake.power)
            telemetry.addLine("-------------------------------------")
            telemetry.addData("Transfer Target Ticks", transfer.targetPosition)
            telemetry.addData("Transfer Current Ticks", transfer.currentPosition)
            telemetry.addLine("-------------------------------------")
            telemetry.addData("Spindexer current state", spindexer.state.name)
            telemetry.addData("Spindexer target angle", spindexer.targetAngle)
            telemetry.addData("Spindexer current angle", spindexer.currentAngle)
            telemetry.addLine("-------------------------------------")
            telemetry.addData("Flywheel target RPM", shooter.targetFlywheelRPM)
            telemetry.addData("Flywheel current RPM", shooter.flywheelRPM)
            telemetry.addData("Hood position", shooter.hoodPosition)
            telemetry.addData("Distance (in)", camera.distanceToGoal)
            telemetry.addLine("-------------------------------------")
            telemetry.addData("April tag current bearing", camera.currentTagBearing)
            telemetry.addData("April tag target bearing", targetTagBearing)
            telemetry.addLine("-------------------------------------")
            telemetry.addData("Turret current", turret.bearing)
            telemetry.addData("Turret target", 0.0)

            telemetry.addData("Loop time", timer.milliseconds())
            timer.reset()
            telemetry.update()
        }
    }
}
package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.utils.LoopTimer
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.ArtifactColorSensor
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Spindexer
import org.firstinspires.ftc.teamcode.subsystems.Transfer
import org.firstinspires.ftc.teamcode.subsystems.Turret
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.OV9281
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.InitConfigurer
import org.firstinspires.ftc.teamcode.util.dpadToAxes
import kotlin.math.abs

//@Configurable
//@TeleOp(name = "Combined TeleOp", group = "TeleOp")
class CombinedTeleOp : LinearOpMode() {
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
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML)
        val drivetrain = Drivetrain().apply {
            fieldCentric = false
        }
        val intake = Intake()
        val transfer = Transfer()
        val spindexer = Spindexer()
        val shooter = Shooter()
        val turret = Turret()
        val camera = OV9281()
        val colorSensor = ArtifactColorSensor()
        val subsystems = listOf(
            drivetrain,
            intake,
            transfer,
            spindexer,
            shooter,
            turret,
            camera,
            colorSensor
        )
        val loopTimer = LoopTimer(10)

        subsystems.forEach { it.initialize() }

        InitConfigurer.preInit()
        while (opModeInInit()) {
            InitConfigurer.postWaitForStart()
        }

        // bulk caching
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        allHubs.forEach { hub -> hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        camera.targetID = InitConfigurer.selectedAlliance.aprilTagID
        loopTimer.start()

        while (opModeIsActive()) {
            // more bulk caching
            allHubs.forEach { hub -> hub.clearBulkCache() }

            // see https://www.padcrafter.com/?templates=Driver+1%7CDriver+2&plat=1%7C1&col=%23242424%2C%23606A6E%2C%23FFFFFF&leftStick=Drivetrain+translation%7CTurret+Manual+Control&rightStick=Drivetrain+rotation%7CSpindexer+Manual+Control&yButton=Actuate+Transfer&xButton=Toggle+intake&rightTrigger=Auto+adjust+shooter%7CTransfer+forward&rightBumper=Spindexer%3A+Next+Outtake&leftBumper=Spindexer%3A+Next+Intake%7CAutoaim+turret&leftTrigger=Reverse+Intake%7CTransfer+backward&bButton=Reset+yaw+%28TO+BE+REMOVED%29%7CLong+shooting+preset&backButton=Toggle+field+centric&dpadLeft=%7CHoodPosition+-+0.05&dpadRight=%7CHoodPosition+%2B+0.05&dpadUp=%7CFlywheel+RPM+%2B+250&dpadDown=%7CFlywheel+RPM+-+250&aButton=%7CClose+shooting+preset
            // for gamepad layouts

            // drivetrain
            if (gamepad1.touchpadWasPressed()) drivetrain.fieldCentric = !drivetrain.fieldCentric
            if (gamepad1.circleWasPressed()) drivetrain.resetYaw()
            drivetrain.setDrivetrainPowers(drivetrain.calculateDrivetrainPowers(
                gamepad1.left_stick_x.toDouble(),
                -gamepad1.left_stick_y.toDouble(),
                gamepad1.right_stick_x.toDouble()))

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

            // map autoaim behind right trigger
            if (gamepad1.right_trigger >= 0.15) {
                turret.bearing = camera.currentTagBearing
            }

            // map auto adjust behind left trigger
            if (gamepad1.left_trigger >= 0.15) {
                shooter.setTargetState(camera.distanceToGoal)
            } else {
                shooter.targetFlywheelRPM = 0.0
            }

            // update all mechanisms
            subsystems.forEach { it.periodic() }

            // if we have a detection, green; otherwise, red
            if (camera.distanceToGoal > 0.0) {
                gamepad1.setLedColor(0.0, 255.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS)
            } else {
                gamepad1.setLedColor(255.0, 0.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS)
            }

            telemetry.addData("Loop Hz", "%05.2f", loopTimer.hz)
            telemetry.addData("Loop ms", "%05.2f", loopTimer.ms.toDouble())
            telemetry.update()
        }
    }
}
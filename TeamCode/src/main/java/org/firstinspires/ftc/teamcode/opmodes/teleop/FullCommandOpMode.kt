package org.firstinspires.ftc.teamcode.opmodes.teleop

import android.util.Log
import com.acmerobotics.dashboard.FtcDashboard
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import dev.nextftc.bindings.button
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.ftc.components.LoopTimeComponent
import org.firstinspires.ftc.teamcode.subsystems.ArtifactColorSensor
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.OV9281
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Spindexer
import org.firstinspires.ftc.teamcode.subsystems.Transfer
import org.firstinspires.ftc.teamcode.subsystems.Turret
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.Artifact
import org.firstinspires.ftc.teamcode.util.BitflipOpMode
import org.firstinspires.ftc.teamcode.util.InitConfigurer
import org.firstinspires.ftc.teamcode.util.MotifPattern
import org.firstinspires.ftc.teamcode.util.TelemetryComponent
import org.firstinspires.ftc.teamcode.util.commands.RepeatCommand
import org.firstinspires.ftc.teamcode.util.commands.RetryCommand
import kotlin.time.Duration.Companion.milliseconds

@TeleOp(name = "Command Drive", group = "TeleOp")
class FullCommandOpMode: BitflipOpMode() {
    val drivetrain = Drivetrain()
    val intake = Intake()
    val camera = OV9281()
    val shooter = Shooter()
    val spindexer = Spindexer()
    val transfer = Transfer()
    val turret = Turret()
    val colorSensor = ArtifactColorSensor()

    var alliance = Alliance.NONE

    fun retryShoot() = RetryCommand(
        transfer.shootArtifact(),
        { !shooter.atSetPoint() },
        3
    ).then(InstantCommand { if (!shooter.atSetPoint()) {spindexer.recordOuttake()}})

    init {
        addComponents(
            SubsystemComponent(
                drivetrain,
                intake,
                camera,
                shooter,
                spindexer,
                transfer,
                turret,
                colorSensor
            ),
            BulkReadComponent,
            BindingsComponent,
            LoopTimeComponent(),
            InitConfigurer
        )
    }

    override fun onInit() {
        telemetry = JoinedTelemetry(telemetry, PanelsTelemetry.ftcTelemetry, FtcDashboard.getInstance().telemetry)

        drivetrain.fieldCentric = true
        shooter.hoodPosition = 0.0
        shooter.periodic()
    }

    override fun onStartButtonPressed() {
        spindexer.motifPattern = camera.motif
        camera.targetID = alliance.aprilTagID

        val drive = LambdaCommand()
            .setUpdate { drivetrain.setDrivetrainPowers(drivetrain.calculateDrivetrainPowers(
                gamepad1.left_stick_x.toDouble(),
                -gamepad1.left_stick_y.toDouble(),
                gamepad1.right_stick_x.toDouble()
            )) }
            .setIsDone { false }
            .setRequirements(drivetrain)
            .setInterruptible(true)
            .setName("Drive")

        drive()

        Gamepads.gamepad1.triangle.whenBecomesTrue(
            SequentialGroup(
                InstantCommand { Log.d("COMMAND_TIMER", "Start time: ${System.nanoTime() / 1000000}")},
                RepeatCommand(
                    SequentialGroup(
                        spindexer.tryMotifOuttake(),
                        WaitUntil(shooter::atSetPoint),
                        retryShoot().thenWait(200.milliseconds)
                    ), spindexer::totalFullSlots
                ).then(spindexer.goToFirstEmptyIntake()),
                InstantCommand { Log.d("COMMAND_TIMER", "End time: ${System.nanoTime() / 1000000}")}
            )
        )

        Gamepads.gamepad1.leftBumper.whenBecomesTrue(
            SequentialGroup (
            InstantCommand { spindexer.recordIntake(colorSensor.detectedArtifact) },
                spindexer.goToFirstEmptyIntake()
            )
        )

        // hold down right bumper for auto indexing
        (Gamepads.gamepad1.rightBumper and button { colorSensor.detectedArtifact != Artifact.NONE }).whenBecomesTrue {
            SequentialGroup (
                InstantCommand { spindexer.recordIntake(colorSensor.detectedArtifact) },
                spindexer.goToFirstEmptyIntake()
            )
        }

        Gamepads.gamepad1.rightTrigger greaterThan 0.15 whenTrue {
            turret.bearing = camera.currentTagBearing
            turret.targetBearing = camera.adjustedTagTargetBearing
            turret.turningPower = gamepad1.right_stick_x.toDouble()
        } whenFalse {
            turret.bearing = 0.0
            turret.turningPower = 0.0
        }

        Gamepads.gamepad1.leftTrigger greaterThan 0.15 whenTrue { shooter.targetFlywheelRPM = 0.0 } whenFalse { shooter.calculateTargetState(camera.distanceToGoal) }

        Gamepads.gamepad1.square whenBecomesTrue intake.toggleRun()
        Gamepads.gamepad1.cross  whenBecomesTrue intake.reverse()   whenBecomesFalse intake.forward()

        Gamepads.gamepad1.circle whenBecomesTrue InstantCommand { drivetrain.resetYaw() }


        Gamepads.gamepad1.dpadLeft whenBecomesTrue ParallelGroup (
            InstantCommand { spindexer.recordIntake(Artifact.GREEN) },
            spindexer.goToFirstEmptyIntake(),
        )

        Gamepads.gamepad1.dpadRight whenBecomesTrue ParallelGroup (
            InstantCommand { spindexer.recordIntake(Artifact.PURPLE) },
            spindexer.goToFirstEmptyIntake(),
        )

        Gamepads.gamepad1.dpadDown whenBecomesTrue spindexer.goToNextOuttake()
        Gamepads.gamepad1.dpadUp whenBecomesTrue transfer.shootArtifact()

        // gamepad red when no atag seen
        button {camera.distanceToGoal > 0.0} whenBecomesTrue InstantCommand {gamepad1.setLedColor(0.0,255.0,0.0,
            Gamepad.LED_DURATION_CONTINUOUS)} whenBecomesFalse InstantCommand {gamepad1.setLedColor(255.0,0.0,0.0,
            Gamepad.LED_DURATION_CONTINUOUS)}
    }

}
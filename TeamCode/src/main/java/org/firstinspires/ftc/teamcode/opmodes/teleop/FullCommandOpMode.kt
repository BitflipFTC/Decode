package org.firstinspires.ftc.teamcode.opmodes.teleop

import android.util.Log
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import dev.nextftc.bindings.button
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import org.firstinspires.ftc.teamcode.util.Artifact
import org.firstinspires.ftc.teamcode.util.BitflipOpMode
import org.firstinspires.ftc.teamcode.util.InitConfigurer
import kotlin.time.Duration.Companion.milliseconds

@TeleOp(name = "Command Drive", group = "TeleOp")
class FullCommandOpMode: BitflipOpMode() {
    init {
        addComponents(
            SubsystemComponent(
                drivetrain.apply {
                    fieldCentric = true
                },
                intake,
                camera,
                shooter,
                spindexer,
                transfer,
                turret,
                colorSensor
            ),
            InitConfigurer
        )
    }

    override fun onStartButtonPressed() {
        spindexer.motifPattern = camera.motif
        camera.targetID = InitConfigurer.selectedAlliance.aprilTagID

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

        // gamepad red when no atag seen
        InstantCommand { gamepad1.setLedColor(255.0,0.0,0.0, Gamepad.LED_DURATION_CONTINUOUS)}
        button {camera.distanceToGoal > 0.0} whenBecomesTrue InstantCommand {gamepad1.setLedColor(0.0,255.0,0.0,
            Gamepad.LED_DURATION_CONTINUOUS)} whenBecomesFalse InstantCommand {gamepad1.setLedColor(255.0,0.0,0.0,
            Gamepad.LED_DURATION_CONTINUOUS)}

        Gamepads.gamepad1.triangle.whenBecomesTrue(
            SequentialGroup(
                InstantCommand { Log.d("COMMAND_TIMER", "Shooting ${spindexer.totalFullSlots} artifacts")},
                InstantCommand { Log.d("COMMAND_TIMER", "Start time: ${System.nanoTime() / 1000000}")},
                shootAllArtifacts(200.milliseconds),
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
        /*(Gamepads.gamepad1.rightBumper and */button { colorSensor.detectedArtifact != Artifact.NONE }/*)*/.whenBecomesTrue {
            SequentialGroup (
                InstantCommand { spindexer.recordIntake(colorSensor.detectedArtifact) },
                spindexer.goToFirstEmptyIntake()
            )
        }

        Gamepads.gamepad1.rightTrigger greaterThan 0.15 whenTrue {
            turret.bearing = camera.currentTagBearing
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
    }

}
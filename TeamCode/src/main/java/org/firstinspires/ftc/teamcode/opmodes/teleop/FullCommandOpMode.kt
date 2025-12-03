package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import dev.nextftc.bindings.button
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.components.LoopTimeComponent
import org.firstinspires.ftc.teamcode.util.Artifact
import org.firstinspires.ftc.teamcode.util.BetterLoopTimeComponent
import org.firstinspires.ftc.teamcode.util.BitflipOpMode
import org.firstinspires.ftc.teamcode.util.InitConfigurer
import kotlin.time.Duration.Companion.milliseconds

@TeleOp(name = "Command Drive", group = "TeleOp")
class FullCommandOpMode : BitflipOpMode() {
    init {
        addComponents(
            SubsystemComponent(
                drivetrain.apply {
                    fieldCentric = false
                },
                intake,
                camera,
                shooter,
                spindexer,
                transfer,
                turret,
                colorSensor
            ),
            InitConfigurer,
            BetterLoopTimeComponent
        )
    }

    override fun onStartButtonPressed() {
        // todo remove these once we have a functional auto that gets these.
        spindexer.motifPattern = camera.motif
        camera.targetID = InitConfigurer.selectedAlliance.aprilTagID

        // Teleop Driver controls
        val drive = LambdaCommand()
            .setUpdate {
                drivetrain.setDrivetrainPowers(
                    drivetrain.calculateDrivetrainPowers(
                        gamepad1.left_stick_x.toDouble(),
                        -gamepad1.left_stick_y.toDouble(),
                        gamepad1.right_stick_x.toDouble()
                    )
                )
            }
            .setIsDone { false }
            .setRequirements(drivetrain)
            .setInterruptible(true)
            .setName("Drive")
        drive()
        Gamepads.gamepad1.circle whenBecomesTrue InstantCommand { drivetrain.resetYaw() }

        // Gamepad lighting control
        InstantCommand { gamepad1.setLedColor(255.0, 0.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS) }()
        button { camera.distanceToGoal > 0.0 } whenBecomesTrue InstantCommand {
            gamepad1.setLedColor(0.0, 255.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS)
        } whenBecomesFalse InstantCommand {
            gamepad1.setLedColor(255.0, 0.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS)
        }

        // Auto shoot all artifacts
        Gamepads.gamepad1.triangle.whenBecomesTrue(
            shootAllArtifacts(200.milliseconds),
        )

        // Auto indexing
        // check that the spindexer isn't full so it doesn't continuously record new intakes when it becomes full
        button { colorSensor.detectedArtifact != Artifact.NONE && !spindexer.isFull }.whenBecomesTrue(
            SequentialGroup(
                InstantCommand { spindexer.recordIntake(colorSensor.detectedArtifact) },
                spindexer.goToFirstEmptyIntake()
            )
        )
        button { spindexer.isFull } whenBecomesTrue spindexer.tryMotifOuttake().and(intake.reverse()) whenBecomesFalse intake.forward()

        // Manual override for auto index
        Gamepads.gamepad1.leftBumper.whenBecomesTrue(
            SequentialGroup(
                InstantCommand { spindexer.recordIntake(colorSensor.detectedArtifact) },
                spindexer.goToFirstEmptyIntake()
            )
        )

        // Turret autoaiming (can turn off with right trigger)
        Gamepads.gamepad1.rightTrigger greaterThan 0.15 whenFalse {
            turret.bearing = camera.currentTagBearing
            turret.turningPower = gamepad1.right_stick_x.toDouble()
        } whenTrue {
            turret.bearing = 0.0
            turret.turningPower = 0.0
        }

        // Shooter auto adjusting (can turn off with left trigger)
        Gamepads.gamepad1.leftTrigger greaterThan 0.15 whenFalse {
            shooter.targetFlywheelRPM = 0.0
        } whenTrue {
            shooter.setTargetState(camera.distanceToGoal)
        }

        // Intake controls
        Gamepads.gamepad1.square whenBecomesTrue intake.toggleRun()
        Gamepads.gamepad1.cross whenBecomesTrue intake.reverse() whenBecomesFalse intake.forward()


        // Manual controls (testing)
        Gamepads.gamepad1.dpadDown whenBecomesTrue ParallelGroup(
            InstantCommand {
                spindexer.recordOuttake(0)
                spindexer.recordOuttake(1)
                spindexer.recordOuttake(2)
            }
        )
        Gamepads.gamepad1.dpadUp whenBecomesTrue transfer.shootArtifact()
        Gamepads.gamepad1.dpadLeft whenBecomesTrue spindexer.goToNextOuttake()
        Gamepads.gamepad1.dpadRight whenBecomesTrue spindexer.goToNextIntake()
    }
}
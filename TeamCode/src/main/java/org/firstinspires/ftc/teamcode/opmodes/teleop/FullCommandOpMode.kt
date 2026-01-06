package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import dev.nextftc.bindings.button
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.Gamepads
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
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
                    driveSpeed = 1.0
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
            BetterLoopTimeComponent,
            PedroComponent(Constants::createFollower)
        )
    }

    override fun onStartButtonPressed() {
        // todo remove these once we have a functional auto that gets these.
        spindexer.motifPattern = camera.motif
//        camera.targetID = InitConfigurer.selectedAlliance?.aprilTagID ?: 24
        turret.selectedAlliance = InitConfigurer.selectedAlliance

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
//        button { camera.distanceToGoal > 0.0 } whenBecomesTrue InstantCommand {
//            gamepad1.setLedColor(0.0, 255.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS)
//        } whenBecomesFalse InstantCommand {
//            gamepad1.setLedColor(255.0, 0.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS)
//        }

        // Auto shoot all artifacts
        Gamepads.gamepad1.triangle.whenBecomesTrue(
            shootAllArtifacts(),
        )

        // Auto indexing
        // check that the spindexer isn't full so it doesn't continuously record new intakes when it becomes full
        button { colorSensor.detectedArtifact != null && !spindexer.isFull }.whenBecomesTrue(
            SequentialGroup(
                InstantCommand { spindexer.recordIntake(colorSensor.detectedArtifact!!) },
                spindexer.goToFirstEmptyIntake()
            )
        )
        button { spindexer.isFull } whenBecomesTrue spindexer.tryMotifOuttake().and(intake.reverse()) whenBecomesFalse intake.forward()

        // Turret autoaiming (can turn off with right trigger)
//        Gamepads.gamepad1.rightTrigger greaterThan 0.15 whenTrue {
//            turret.robotPose = follower.pose
//        }

        // Shooter auto adjusting (can turn off with left trigger)
//        Gamepads.gamepad1.leftTrigger greaterThan 0.15 whenFalse {
//            shooter.targetFlywheelRPM = 0.0
//        } whenTrue {
//            shooter.setTargetState(camera.distanceToGoal)
//        }

        // Intake controls
        Gamepads.gamepad1.square whenBecomesTrue intake.toggleRun()
        Gamepads.gamepad1.cross whenBecomesTrue intake.reverse() whenBecomesFalse intake.forward()

        // Manual controls (testing)
        Gamepads.gamepad1.dpadUp whenBecomesTrue InstantCommand { shooter.targetFlywheelRPM += 250.0}
        Gamepads.gamepad1.dpadDown whenBecomesTrue InstantCommand { shooter.targetFlywheelRPM -= 250.0 }
        Gamepads.gamepad1.dpadRight whenBecomesTrue InstantCommand { shooter.hoodPosition += 0.5}
        Gamepads.gamepad1.dpadLeft whenBecomesTrue InstantCommand { shooter.hoodPosition -= 0.5 }
    }
}
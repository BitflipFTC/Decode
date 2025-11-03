package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.CommandScheduler
import com.seattlesolvers.solverslib.command.ConditionalCommand
import com.seattlesolvers.solverslib.command.FunctionalCommand
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.RetryCommand
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.command.WaitUntilCommand
import com.seattlesolvers.solverslib.command.button.Trigger
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.gamepad.GamepadKeys
import com.seattlesolvers.solverslib.gamepad.toggleWhenActive
import org.firstinspires.ftc.teamcode.hardware.Drivetrain
import org.firstinspires.ftc.teamcode.hardware.Intake
import org.firstinspires.ftc.teamcode.hardware.OV9281
import org.firstinspires.ftc.teamcode.hardware.Shooter
import org.firstinspires.ftc.teamcode.hardware.Spindexer
import org.firstinspires.ftc.teamcode.hardware.Transfer
import org.firstinspires.ftc.teamcode.hardware.Turret
import org.firstinspires.ftc.teamcode.util.Artifact
import org.firstinspires.ftc.teamcode.util.MotifPattern
import java.util.stream.IntStream

@TeleOp(name = "Command Drive", group = "TeleOp")
class FullCommandOpMode: CommandOpMode() {
    lateinit var drivetrain: Drivetrain
    lateinit var intake: Intake
    lateinit var camera: OV9281
    lateinit var shooter: Shooter
    lateinit var spindexer: Spindexer
    lateinit var transfer: Transfer
    lateinit var turret: Turret
    var totalLoops = 0
    val timer = ElapsedTime()

    override fun initialize() {
        telemetry = JoinedTelemetry(telemetry, PanelsTelemetry.ftcTelemetry, FtcDashboard.getInstance().telemetry)
        telemetry.msTransmissionInterval = 500
        drivetrain = Drivetrain(this)
        intake     = Intake(this)
        camera     = OV9281(this, 4, 6)
        shooter    = Shooter(this)
        spindexer  = Spindexer(this)
        transfer   = Transfer(this)
        turret     = Turret(this)
        val g1 = GamepadEx(gamepad1)
        val g2 = GamepadEx(gamepad2)
        val runIntake = RunCommand(intake::intake, intake)
        val stopIntake = RunCommand(intake::off, intake)
        val slowIntake = RunCommand(intake::slow, intake)
        val outtake = RunCommand(intake::outtake, intake)

        camera.targetID = 21
        spindexer.motifPattern = MotifPattern.GPP

        intake.defaultCommand = slowIntake

        g1.getGamepadButton(GamepadKeys.Button.SQUARE).toggleWhenPressed(runIntake)
        g1.getGamepadButton(GamepadKeys.Button.CROSS).whenHeld(outtake)
        g1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(stopIntake)

        turret.defaultCommand = RunCommand({
            turret.bearing = camera.currentTagBearing
        }, turret)

        shooter.defaultCommand = RunCommand({
            shooter.calculateTargetState(camera.distanceToGoal)
        }, shooter)

        val disableAutoaim = Trigger { g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.25 }
        disableAutoaim.whileActiveContinuous(
            InstantCommand(
                {
                    turret.bearing = 0.0
                    shooter.targetFlywheelRPM = 0.0
                }, turret, shooter
            )
        )

        val aprilTagSeen = Trigger { camera.distanceToGoal <= 0.0 }
        aprilTagSeen.toggleWhenActive(
            InstantCommand({
                gamepad1.setLedColor(0.0,255.0,0.0,Gamepad.LED_DURATION_CONTINUOUS)
            }),
            InstantCommand({
                gamepad1.setLedColor(255.0,0.0,0.0, Gamepad.LED_DURATION_CONTINUOUS)
            })
        )

        drivetrain.defaultCommand = RunCommand({
            drivetrain.setDrivetrainPowers(drivetrain.calculateDrivetrainPowers(
                g1.leftX,
                g1.leftY,
                g1.rightX
            ))
        }, drivetrain)

        val shootBall = FunctionalCommand(
            transfer::transferArtifact,
            {}, { interrupted -> {} },
            transfer::atSetPoint,
            transfer
        )

        val waitThenShootBall = SequentialCommandGroup(
            WaitUntilCommand(shooter::atSetPoint),
            shootBall
        )

        val shootBallRetry = RetryCommand(
            waitThenShootBall,
            { !shooter.atSetPoint() }, // ensures artifact was actually launched
            3
        ).whenFinished { if (!shooter.atSetPoint()) { spindexer.recordOuttake() } }
        // whenFinished evaluates the logic in the same loop of when the RetryCommand ends,
        // so !shooter.atSetPoint() will have the same value when polled in the .whenFinished{} bloc
        // and when polled in the success condition

        val goToNextIntake = FunctionalCommand(
            spindexer::toNextIntakePosition,
            {}, { interrupted -> {} },
            spindexer::atSetPoint,
            spindexer
        )

        val goToNextOuttake = FunctionalCommand(
            spindexer::toNextOuttakePosition,
            {}, { interrupted -> {}},
            spindexer::atSetPoint,
            spindexer
        )

        val goToFirstFullOuttake = FunctionalCommand(
            spindexer::toFirstFullOuttakePosition,
            {}, { interrupted -> {} },
            spindexer::atSetPoint,
            spindexer, intake
        )

        val goToMotifOuttake = FunctionalCommand(
            spindexer::toMotifOuttakePosition,
            {}, { interrupted -> {} },
            spindexer::atSetPoint,
            spindexer
        )

        val tryMotifOuttake = ConditionalCommand(
            goToMotifOuttake,
            goToFirstFullOuttake,
            spindexer::hasMotifAssortment
        )

        val shootCycle = SequentialCommandGroup(
            WaitUntilCommand(turret::atSetPoint).withTimeout(1000),
            shootBallRetry,
            // if the spindexer is empty, return to intake 0.
            ConditionalCommand(
                goToNextIntake,
                goToNextOuttake,
                spindexer::isEmpty
            )
        )

        // returns to intake if it's empty
        val shootAllArtifacts = SequentialCommandGroup(
            tryMotifOuttake,
            RetryCommand(
                shootCycle,
                { spindexer.fullSlotNumber == 0 },
                3
            )
        )

        g1.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
            shootAllArtifacts
        )

        g1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
            InstantCommand(
                { spindexer.recordIntake(Artifact.GREEN); spindexer.toNextIntakePosition() },
                spindexer
            )
        )

        g1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
            InstantCommand(
                { spindexer.recordIntake(Artifact.PURPLE); spindexer.toNextIntakePosition() },
                spindexer
            )
        )

        CommandScheduler.getInstance().setBulkReading(hardwareMap, LynxModule.BulkCachingMode.MANUAL)
    }

    override fun run() {
        super.run()
        totalLoops++
        telemetry.addData("Avg loop time", timer.milliseconds() / totalLoops)
        if (totalLoops == 500) {
            timer.reset()
            totalLoops = 0
        }

        telemetry.addData("Times to shoot", spindexer.fullSlotNumber)

        telemetry.update()
    }
}
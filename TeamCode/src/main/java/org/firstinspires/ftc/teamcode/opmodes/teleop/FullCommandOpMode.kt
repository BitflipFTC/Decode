package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.FunctionalCommand
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.ParallelCommandGroup
import com.seattlesolvers.solverslib.command.RepeatCommand
import com.seattlesolvers.solverslib.command.RetryCommand
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.command.WaitUntilCommand
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.gamepad.GamepadKeys
import org.firstinspires.ftc.teamcode.hardware.Drivetrain
import org.firstinspires.ftc.teamcode.hardware.Intake
import org.firstinspires.ftc.teamcode.hardware.OV9281
import org.firstinspires.ftc.teamcode.hardware.Shooter
import org.firstinspires.ftc.teamcode.hardware.Spindexer
import org.firstinspires.ftc.teamcode.hardware.Transfer
import org.firstinspires.ftc.teamcode.hardware.Turret
import org.firstinspires.ftc.teamcode.util.Artifact

@TeleOp(name = "Command Drive", group = "TeleOp")
class FullCommandOpMode: CommandOpMode() {

    override fun initialize() {
        telemetry = JoinedTelemetry(telemetry, PanelsTelemetry.ftcTelemetry, FtcDashboard.getInstance().telemetry)
        val drivetrain = Drivetrain(this)
        val intake = Intake(this)
        val camera = OV9281(this, 4, 6)
        val shooter = Shooter(this)
        val spindexer = Spindexer(this)
        val transfer = Transfer(this)
        val turret = Turret(this)
        val g1 = GamepadEx(gamepad1)
        val intakeCommand = RunCommand(intake::intake, intake)

        camera.targetID = 21
        spindexer.recordIntake(Artifact.GREEN, 0)
        spindexer.recordIntake(Artifact.PURPLE,1)
        spindexer.recordIntake(Artifact.PURPLE,2)

        intake.defaultCommand = RunCommand(intake::slow, intake)
        g1.getGamepadButton(GamepadKeys.Button.SQUARE).toggleWhenPressed(intakeCommand)

        g1.getGamepadButton(GamepadKeys.Button.CROSS).whenHeld(
            RunCommand(intake::outtake, intake).whenFinished {intakeCommand},
        )

        g1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
            RunCommand(intake::off, intake)
        )

        turret.defaultCommand = RunCommand({
            turret.bearing = camera.currentTagBearing
        }, turret)

        shooter.defaultCommand = RunCommand({
            shooter.calculateTargetState(camera.distanceToGoal)
        }, shooter)

        drivetrain.defaultCommand = RunCommand({
            drivetrain.setDrivetrainPowers(drivetrain.calculateDrivetrainPowers(
                g1.leftX,
                g1.leftY,
                g1.rightX
            ))
        }, drivetrain)

        g1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
            FunctionalCommand(
                spindexer::toNextOuttakePosition,
                {},
                { interrupted -> {} },
                spindexer::atSetPoint,
                spindexer
            )
        )

        g1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
            FunctionalCommand(
                spindexer::toNextIntakePosition,
                {},
                { interrupted -> {} },
                spindexer::atSetPoint,
                spindexer
            )
        )

        val shootBall = FunctionalCommand(
            transfer::transferArtifact,
            {},
            { interrupted -> {} },
            transfer::atSetPoint,
            transfer
        )

        val shootBallRetry = RetryCommand(
            shootBall,
            { !shooter.atSetPoint() },
            3
        )

        val goToNextOuttake = FunctionalCommand(
            spindexer::toNextOuttakePosition,
            {},
            { interrupted -> {}},
            spindexer::atSetPoint,
            spindexer
        )

        val goToFirstOuttake = FunctionalCommand(
            { spindexer.state = Spindexer.States.OUTTAKE_ZERO },
            {},
            { interrupted -> {}},
            spindexer::atSetPoint,
            spindexer
        )

        val waitForTurret = WaitUntilCommand(
            turret::atSetPoint
        )

        val waitForFlywheel = WaitUntilCommand(
            shooter::atSetPoint
        )

        val waitUntilReady = ParallelCommandGroup(
            waitForTurret,
            waitForFlywheel
        )

        val shootCycle = SequentialCommandGroup(
            waitUntilReady,
            shootBallRetry,
            goToNextOuttake
        )

        val shootThreeBalls = SequentialCommandGroup(
            goToFirstOuttake,
            RepeatCommand(shootCycle, 3),
            InstantCommand(spindexer::toNextIntakePosition, spindexer)
        )

        g1.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
            shootThreeBalls
        )
    }

    override fun run() {
        super.run()

        telemetry.update()
    }
}
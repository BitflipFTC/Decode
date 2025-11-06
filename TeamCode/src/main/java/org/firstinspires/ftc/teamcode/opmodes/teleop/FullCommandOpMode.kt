package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.CommandGroupBase
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.CommandScheduler
import com.seattlesolvers.solverslib.command.ConditionalCommand
import com.seattlesolvers.solverslib.command.FunctionalCommand
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.ParallelCommandGroup
import com.seattlesolvers.solverslib.command.ParallelRaceGroup
import com.seattlesolvers.solverslib.command.RepeatCommand
import com.seattlesolvers.solverslib.command.RetryCommand
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.command.ScheduleCommand
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.command.StartEndCommand
import com.seattlesolvers.solverslib.command.WaitUntilCommand
import com.seattlesolvers.solverslib.command.button.Trigger
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.gamepad.GamepadKeys
import com.seattlesolvers.solverslib.gamepad.whenInactive
import com.seattlesolvers.solverslib.gamepad.whileActiveOnce
import org.firstinspires.ftc.teamcode.hardware.Drivetrain
import org.firstinspires.ftc.teamcode.hardware.Intake
import org.firstinspires.ftc.teamcode.hardware.OV9281
import org.firstinspires.ftc.teamcode.hardware.Shooter
import org.firstinspires.ftc.teamcode.hardware.Spindexer
import org.firstinspires.ftc.teamcode.hardware.Transfer
import org.firstinspires.ftc.teamcode.hardware.Turret
import org.firstinspires.ftc.teamcode.util.Artifact
import org.firstinspires.ftc.teamcode.util.MotifPattern

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

    fun shootAllArtifacts(num: Int) = SequentialCommandGroup(
        spindexer.tryMotifOuttake(),
        RepeatCommand(
            shootCycle(),
            num
        )
    )

    fun shootBallRetry(): Command = RetryCommand(
        SequentialCommandGroup(
            WaitUntilCommand(shooter::atSetPoint),
            transfer.shootArtifact()
        ),
        { !shooter.atSetPoint() }, // ensures artifact was actually launched
        3
    ).whenFinished { if (!shooter.atSetPoint()) { spindexer.recordOuttake() } }
    // whenFinished evaluates the logic in the same loop of when the RetryCommand ends,
    // so !shooter.atSetPoint() will have the same value when polled in the .whenFinished{} bloc
    // and when polled in the success condition

    fun shootCycle() = SequentialCommandGroup(
        WaitUntilCommand(turret::atSetPoint).withTimeout(1000),
        shootBallRetry(),

        // if the spindexer is empty, return to empty intake.
        ConditionalCommand(
            spindexer.goToFirstEmptyIntake(),
            spindexer.goToFirstFullOuttake(),
            spindexer::isEmpty
        )
    )

    fun shootCycleFast() = SequentialCommandGroup(
        spindexer.tryMotifOuttake(),
        InstantCommand({transfer.setPower(1.0)}),
            spindexer.goToNextOuttake(),
            spindexer.goToNextOuttake(),
            spindexer.goToNextOuttake(),
            spindexer.goToNextOuttake(),
        InstantCommand({transfer.setPower(0.0)})
    )

    override fun initialize() {
        telemetry = JoinedTelemetry(telemetry, PanelsTelemetry.ftcTelemetry, FtcDashboard.getInstance().telemetry)
        telemetry.msTransmissionInterval = 500
        drivetrain = Drivetrain(this)
        drivetrain.fieldCentric = false

        intake     = Intake(this)
        camera     = OV9281(this, 4, 6)
        shooter    = Shooter(this)
        spindexer  = Spindexer(this)
        transfer   = Transfer(this)
        turret     = Turret(this)
        val g1 = GamepadEx(gamepad1)

        camera.targetID = 21
        spindexer.motifPattern = MotifPattern.GPP

        g1.getGamepadButton(GamepadKeys.Button.SQUARE).toggleWhenPressed(intake.stopIntake())
        g1.getGamepadButton(GamepadKeys.Button.CROSS).whileHeld(intake.runOuttake()).whenReleased(ConditionalCommand(
            intake.runIntake(),
            intake.stopIntake(),
            { intake.power == Intake.State.INTAKE }
        ))

        val enableAutoaim = Trigger { g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.2 }
        enableAutoaim.whenActive(
            RunCommand(
                {
                    turret.bearing = camera.currentTagBearing
                    shooter.calculateTargetState(camera.distanceToGoal)
                }, turret, shooter
            )
        ).whenInactive(
            InstantCommand(
                {
                    turret.bearing = 0.0
                    shooter.state = Shooter.ShooterState(0.0,0.0)
                }, turret, shooter
            )
        )

        // todo test led logic
        val aprilTagSeen = Trigger { camera.detectionsAmount >= 1 }
        aprilTagSeen.whenActive(
            InstantCommand({
                gamepad1.setLedColor(0.0,255.0,0.0,Gamepad.LED_DURATION_CONTINUOUS)
            })
        ).whenInactive(
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

        // returns to intake if it's empty
        g1.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
            shootAllArtifacts(spindexer.totalFullSlots)
        )

        g1.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
            shootCycleFast()
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

//        g1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).toggleWhenPressed(
//            StartEndCommand(
//                {
//                    camera.visionPortal.setProcessorEnabled(camera.aprilTag, false)
//                    camera.resetAprilTagData()
//                },
//                { camera.visionPortal.setProcessorEnabled(camera.aprilTag, true) },
//                camera
//            )
//        )

        CommandScheduler.getInstance().setBulkReading(hardwareMap, LynxModule.BulkCachingMode.MANUAL)
    }

    enum class InitializeState {
        GET_MOTIF,
    }

    var initializeState = InitializeState.GET_MOTIF
    var selectedMotif = MotifPattern.NONE
    val allMotifs = MotifPattern.entries.toTypedArray()
    override fun initialize_loop() {
        super.initialize_loop()

        when (initializeState) {
            InitializeState.GET_MOTIF -> {
                when (selectedMotif) {
                    MotifPattern.GPP  -> {
                        telemetry.addLine("- GPP")
                        telemetry.addLine("PGP")
                        telemetry.addLine("PPG")
                        telemetry.addLine("NONE")

                    }
                    MotifPattern.PGP  -> {
                        telemetry.addLine("GPP")
                        telemetry.addLine("- PGP")
                        telemetry.addLine("PPG")
                        telemetry.addLine("NONE")
                    }
                    MotifPattern.PPG  -> {
                        telemetry.addLine("GPP")
                        telemetry.addLine("PGP")
                        telemetry.addLine("- PPG")
                        telemetry.addLine("NONE")
                    }
                    MotifPattern.NONE -> {
                        telemetry.addLine("GPP")
                        telemetry.addLine("PGP")
                        telemetry.addLine("PPG")
                        telemetry.addLine("- NONE")
                    }
                }

                if (gamepad1.dpadUpWasPressed()) {
                    selectedMotif = if (selectedMotif.ordinal == 0) allMotifs[0] else allMotifs[selectedMotif.ordinal - 1]
                }

                if (gamepad1.dpadDownWasPressed()) {
                    selectedMotif = if (selectedMotif.ordinal == 3) allMotifs[3] else allMotifs[selectedMotif.ordinal + 1]
                }

                if (gamepad1.crossWasPressed()) {
                    spindexer.motifPattern = selectedMotif
                }
            }
        }

        telemetry.update()
    }

    override fun run() {
        super.run()
        totalLoops++
        telemetry.addData("Avg loop time", timer.milliseconds() / totalLoops)

        telemetry.update()
    }
}
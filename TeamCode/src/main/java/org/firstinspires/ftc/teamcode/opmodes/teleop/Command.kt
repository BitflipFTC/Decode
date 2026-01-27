package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.utils.LoopTimer
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.PerpetualCommand
import com.seattlesolvers.solverslib.command.RepeatCommand
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.command.WaitUntilCommand
import org.firstinspires.ftc.teamcode.opmodes.teleop.CombinedTeleOp.Companion.follower
import org.firstinspires.ftc.teamcode.opmodes.teleop.CombinedTeleOp.Companion.motifPattern
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.OV9281
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Spindexer
import org.firstinspires.ftc.teamcode.subsystems.Transfer
import org.firstinspires.ftc.teamcode.subsystems.Turret
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.auto.AutoPoses
import org.firstinspires.ftc.teamcode.util.components.BetterLoopTimeComponent
import org.firstinspires.ftc.teamcode.util.components.InitConfigurer

class Command : CommandOpMode() {
    fun shoot() = RepeatCommand(
        SequentialCommandGroup(
            spindexer.tryMotifOuttake().withTimeout(500),
            WaitUntilCommand(shooter::atSetPoint).withTimeout(1000),
            transfer.commandShoot()
        ), spindexer.totalFullSlots
    ).andThen(spindexer.goToNextIntake())

    fun drive() = RunCommand(
        {
            fol.setTeleOpDrive(
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble(),
                true
            )
        }
    )

    lateinit var intake: Intake
    lateinit var transfer: Transfer
    lateinit var spindexer: Spindexer
    lateinit var shooter: Shooter
    lateinit var turret: Turret
    lateinit var camera: OV9281
    lateinit var colorSensor: ColorSensor
    val subsystems = listOf(
        intake,
        transfer,
        spindexer,
        shooter,
        turret,
        camera,
        colorSensor
    )
    val components = setOf(BetterLoopTimeComponent, InitConfigurer)

    lateinit var fol: Follower
    lateinit var autoPoses: AutoPoses
    lateinit var goToPark: () -> PathChain

    var automatedDriving = false
    var posesHaveBeenBuilt = false


    override fun initialize() {
        super.reset()
        intake = Intake()
        transfer = Transfer()
        spindexer = Spindexer()
        shooter = Shooter()
        turret = Turret()
        camera = OV9281()
        colorSensor = ColorSensor()

        fol = follower ?: Constants.createFollower(hardwareMap).apply {
            setStartingPose(Pose(72.0,72.0,Math.toRadians(90.0)))
            follower = this
        }

        spindexer.motifPattern = motifPattern
        gamepad1.triggerThreshold = 0.15f
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
        components.forEach { it.init() }
        fol.update()
        Drawing.init()
    }

    override fun initialize_loop() {
        if (!InitConfigurer.hasSelectedAlliance) {
            components.forEach { it.initLoop() }
            telemetry.update()
            fol.update()
        } else if (!posesHaveBeenBuilt) {
            autoPoses = AutoPoses(InitConfigurer.selectedAlliance ?: Alliance.RED)
            goToPark = {
                fol.pathBuilder()
                    .addPath(
                        BezierLine(
                            fol::getPose,
                            if (InitConfigurer.selectedAlliance == Alliance.RED) autoPoses.redPark else autoPoses.redPark.mirror()
                        )
                    )
                    .setConstantHeadingInterpolation(autoPoses.redPark.heading)
                    .build()
            }
            posesHaveBeenBuilt = true

        }
    }

    override fun end() {
        super.end()
    }
}
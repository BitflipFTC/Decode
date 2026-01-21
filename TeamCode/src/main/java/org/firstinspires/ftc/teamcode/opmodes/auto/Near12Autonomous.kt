package org.firstinspires.ftc.teamcode.opmodes.auto

import android.util.Log
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.opmodes.auto.paths.Path12
import org.firstinspires.ftc.teamcode.opmodes.teleop.CombinedTeleOp
import org.firstinspires.ftc.teamcode.opmodes.teleop.CombinedTeleOp.Companion.follower
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing
import org.firstinspires.ftc.teamcode.subsystems.ArtifactColorSensor
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.OV9281
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Spindexer
import org.firstinspires.ftc.teamcode.subsystems.Transfer
import org.firstinspires.ftc.teamcode.subsystems.Turret
import org.firstinspires.ftc.teamcode.util.Artifact
import org.firstinspires.ftc.teamcode.util.components.BetterLoopTimeComponent
import org.firstinspires.ftc.teamcode.util.FiniteStateMachine
import org.firstinspires.ftc.teamcode.util.components.InitConfigurer
import org.firstinspires.ftc.teamcode.util.followCustomPath

@Suppress("UNUSED")
@Autonomous(name = "12 ball near", preselectTeleOp = "Combined TeleOp")
class Near12Autonomous : LinearOpMode() {
    enum class Shoot {
        IDLE,
        MOVE_SPINDEXER,
        TRANSFER_ARTIFACT,
        WAIT_FOR_COMPLETION
    }

    private var shootingState = Shoot.IDLE


    val components = setOf(BetterLoopTimeComponent, InitConfigurer)

    val finiteStateMachine: FiniteStateMachine = FiniteStateMachine()
    lateinit var spindexer: Spindexer
    lateinit var turret: Turret
    lateinit var transfer: Transfer
    lateinit var shooter: Shooter
    lateinit var intake: Intake
    lateinit var colorSensor: ArtifactColorSensor
    lateinit var camera: OV9281

    override fun runOpMode() {
        spindexer = Spindexer()
        turret = Turret()
        transfer = Transfer()
        shooter = Shooter()
        intake = Intake()
        colorSensor = ArtifactColorSensor()
        camera = OV9281()
        val subsystems = setOf(spindexer, turret, transfer, shooter, intake, colorSensor)

        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)

        follower = Constants.createFollower(hardwareMap)

        components.forEach { it.init() }

        Log.d("FSM", "auto inited")
        while (opModeInInit() && !InitConfigurer.hasSelectedAlliance) {
            components.forEach { it.initLoop() }
            follower!!.update()
            telemetry.update()
        }

        val pathSequence = Path12(InitConfigurer.selectedAlliance!!)
        turret.selectedAlliance = InitConfigurer.selectedAlliance!!
        follower!!.pose = pathSequence.poses.nearStartPose
        val paths = pathSequence.buildPaths(follower!!)

        while (opModeInInit()) {
            follower!!.update()
            telemetry.update()
        }


        // set up preloads
        spindexer.setCollectedArtifacts(
            Artifact.GREEN,
            Artifact.PURPLE,
            Artifact.PURPLE,
        )

        finiteStateMachine
            .addState(
                "DScore Preload",
                ::opModeIsActive,
                {
                    follower!!.followCustomPath(paths[0])
                    intake.intake()
                }
            ).addState(
                "Shoot preload",
                {!follower!!.isBusy },
                {
                    spindexer.motifPattern = camera.motif
                    CombinedTeleOp.motifPattern = spindexer.motifPattern
                    Log.d("FSM", "motif pattern detected: ${spindexer.motifPattern}")
                    shootAllArtifacts()
                }
            ).addState(
                "DIntake 1",
                {shootingState == Shoot.IDLE},
                {follower!!.followCustomPath(paths[1])}
            ).addState(
                "Intake 1",
                {!follower!!.isBusy},
                {follower!!.followCustomPath(paths[2])}
            ).addState(
                "DScore 1",
                {!follower!!.isBusy},
                {follower!!.followCustomPath(paths[3])}
            ).addState(
                "Shoot 1",
                {!follower!!.isBusy},
                ::shootAllArtifacts
            ).addState(
                "DIntake 2",
                {shootingState == Shoot.IDLE},
                {follower!!.followCustomPath(paths[4])}
            ).addState(
                "Intake 2",
                {!follower!!.isBusy},
                {follower!!.followCustomPath(paths[5])}
            ).addState(
                "DScore 2",
                {!follower!!.isBusy},
                {follower!!.followCustomPath(paths[6])}
            ).addState(
                "Shoot 2",
                {!follower!!.isBusy},
                ::shootAllArtifacts
            ).addState(
                "DIntake 3",
                {shootingState == Shoot.IDLE},
                {follower!!.followCustomPath(paths[7])}
            ).addState(
                "Intake 3",
                {!follower!!.isBusy},
                {follower!!.followCustomPath(paths[8])}
            ).addState(
                "DScore 3",
                {!follower!!.isBusy},
                {follower!!.followCustomPath(paths[9])}
            ).addState(
                "Shoot 3",
                {!follower!!.isBusy},
                ::shootAllArtifacts
            ).addState(
                "Park",
                {shootingState == Shoot.IDLE},
                {follower!!.followCustomPath(paths[10])}
            )

        while (opModeIsActive()) {
            finiteStateMachine.run()
            updateShootingFSM()

            val artifactDetected =
                colorSensor.detectedArtifact != null && !spindexer.isFull && spindexer.slotsToIntakes.contains(spindexer.state) && spindexer.atSetPoint()
            if (artifactDetected) {
                spindexer.recordIntake(colorSensor.detectedArtifact!!)
                spindexer.toFirstEmptyIntakePosition()
                Log.d("FSM", "DETECTED ARTIFACT ${colorSensor.detectedArtifact!!}")
            }

            shooter.setTargetState(turret.goalPose.distanceFrom(follower!!.pose))
            turret.robotPose = follower!!.pose

            follower!!.update()
            telemetry.addData("x", follower!!.pose.x)
            telemetry.addData("y", follower!!.pose.y)
            telemetry.addData("heading", follower!!.pose.heading)
            subsystems.forEach { it.periodic() }
            Drawing.drawDebug(follower)
            telemetry.update()

            components.forEach { it.periodic() }
        }

        follower!!.breakFollowing()
    }

    val timer = ElapsedTime()

    fun shootAllArtifacts() {
        if (!spindexer.isEmpty) {
            shootingState = Shoot.MOVE_SPINDEXER
            Log.d("FSM", "Shooting: spindexer has ${spindexer.getArtifactString()}, motif is ${spindexer.motifPattern}")
        }
    }

    fun updateShootingFSM() {
        when (shootingState) {
            Shoot.MOVE_SPINDEXER      -> {
                spindexer.toMotifOuttakePosition()
                shootingState = Shoot.TRANSFER_ARTIFACT
            }

            Shoot.TRANSFER_ARTIFACT   -> {
                if (shooter.atSetPoint() && spindexer.atSetPoint()) {
                    transfer.transferArtifact()
                    shootingState = Shoot.WAIT_FOR_COMPLETION
                }
            }

            Shoot.WAIT_FOR_COMPLETION -> {
                if (transfer.atSetPoint()) {
                    spindexer.recordOuttake()

                    if (spindexer.isEmpty) {
                        shootingState = Shoot.IDLE
                        spindexer.toFirstEmptyIntakePosition()
                    } else {
                        shootingState = Shoot.MOVE_SPINDEXER
                    }
                }
            }

            Shoot.IDLE                -> {}
        }
    }
}
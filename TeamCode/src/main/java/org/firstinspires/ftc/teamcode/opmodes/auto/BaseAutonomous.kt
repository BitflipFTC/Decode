package org.firstinspires.ftc.teamcode.opmodes.auto

import android.util.Log
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import com.skeletonarmy.marrow.prompts.OptionPrompt
import com.skeletonarmy.marrow.prompts.Prompt
import com.skeletonarmy.marrow.prompts.Prompter
import org.firstinspires.ftc.teamcode.opmodes.teleop.CombinedTeleOp.Companion.alliance
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
import org.firstinspires.ftc.teamcode.util.Artifact
import org.firstinspires.ftc.teamcode.util.BetterLoopTimeComponent
import org.firstinspires.ftc.teamcode.util.FiniteStateMachine
import org.firstinspires.ftc.teamcode.util.InitConfigurer
import org.firstinspires.ftc.teamcode.util.InitializeState
import org.firstinspires.ftc.teamcode.util.InstantState
import org.firstinspires.ftc.teamcode.util.State
import org.firstinspires.ftc.teamcode.util.TelemetryImplUpstreamSubmission
import org.firstinspires.ftc.teamcode.util.auto.BaseAutoPath
import org.firstinspires.ftc.teamcode.util.auto.Path

@Suppress("UNUSED")
abstract class BaseAutonomous : LinearOpMode() {
    abstract fun initialize(alliance: Alliance)

    private enum class Shoot {
        IDLE,
        MOVE_SPINDEXER,
        TRANSFER_ARTIFACT,
        WAIT_FOR_COMPLETION
    }

    private var shootingState = Shoot.IDLE

    val spindexer: Spindexer = Spindexer()
    val turret: Turret = Turret()
    val transfer: Transfer = Transfer()
    val shooter: Shooter = Shooter()
    val intake: Intake = Intake()
    val colorSensor: ColorSensor = ColorSensor()
    val camera: OV9281 = OV9281()

    val subsystems = setOf(spindexer, turret, transfer, shooter, intake, colorSensor)

    protected lateinit var pathSequence: BaseAutoPath
    protected lateinit var finiteStateMachine: FiniteStateMachine
    protected lateinit var paths: List<Path>

    override fun runOpMode() {
        telemetry =
            JoinedTelemetry(PanelsTelemetry.ftcTelemetry, TelemetryImplUpstreamSubmission(this))

        follower = Constants.createFollower(hardwareMap)

        subsystems.forEach { it.initialize() }
        camera.initialize()

        Log.d("FSM", "auto inited")
        val prompter = Prompter(this)

        prompter
            .prompt("alliance", OptionPrompt("Select Alliance", Alliance.RED, Alliance.BLUE))
            .onComplete {
                alliance = prompter.get<Alliance>("alliance")
                initialize(alliance!!)
                turret.selectedAlliance = alliance!!
                follower!!.pose = pathSequence.poses.nearStartPose
            }

        while (opModeInInit()) {
            prompter.run()
            follower!!.update()
            telemetry.update()
        }

        if (opModeIsActive()) {
            BetterLoopTimeComponent.preStartButtonPressed()
            // set up preloads
            spindexer.setCollectedArtifacts(
                Artifact.GREEN,
                Artifact.PURPLE,
                Artifact.PURPLE,
            )
        }

        while (opModeIsActive()) {
            // in this order to let spindexer call a periodic
            updateShootingFSM()
            finiteStateMachine.run()
            if (motifPattern == null) {
                motifPattern = camera.motif
                spindexer.motifPattern = motifPattern
            }

            val artifactDetected =
                colorSensor.detectedArtifact != null && !spindexer.isFull && spindexer.slotsToIntakes.contains(
                    spindexer.state
                ) && spindexer.atSetPoint()
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
            BetterLoopTimeComponent.postUpdate()
        }

        follower!!.breakFollowing()
    }

    private val timer = ElapsedTime()

    protected fun shootAllArtifacts() {
        if (!spindexer.isEmpty) {
            shootingState = Shoot.MOVE_SPINDEXER
            Log.d(
                "FSM",
                "Shooting: spindexer has ${spindexer.getArtifactString()}, motif is ${spindexer.motifPattern}"
            )
        }
    }

    private fun updateShootingFSM() {
        when (shootingState) {
            Shoot.MOVE_SPINDEXER      -> {
                spindexer.toMotifOuttakePosition()
                Log.d(
                    "FSM",
                    "MOVING SPINDEXER TO ${spindexer.state.name}, ${spindexer.getArtifactString()}"
                )
                shootingState = Shoot.TRANSFER_ARTIFACT
                timer.reset()
            }

            Shoot.TRANSFER_ARTIFACT -> {
//                Log.d("FSM", "Waiting for shooter or spindexer")
//                Log.d("FSM", "sp: ${spindexer.currentAngle}, ${spindexer.targetAngle}, sh: ${shooter.flywheelRPM}, ${shooter.targetFlywheelRPM}")
                if ((shooter.atSetPoint()) && spindexer.atSetPoint()) {
                    Log.d("FSM", "-------- Moving spindexer / shooter took ${timer.milliseconds()}")
                    transfer.transferArtifact()
                    Log.d("FSM", "TRANSFERING")
                    shootingState = Shoot.WAIT_FOR_COMPLETION
                    timer.reset()
                }
            }

            Shoot.WAIT_FOR_COMPLETION -> {
//                Log.d("FSM", "WAITING FOR TRANSFER, current: ${transfer.currentPosition}, target: ${transfer.targetPosition}, diff: ${transfer.targetPosition - transfer.currentPosition}")
                if (transfer.atSetPoint()) {
                    Log.d("FSM", "------- transferring took ${timer.milliseconds()}")
                    spindexer.recordOuttake()
                    Log.d("FSM", "EVALUATING SPINDEXER FULLNESS")
                    Log.d(
                        "FSM",
                        "Spindexer isEmpty: " + spindexer.isEmpty + ", isFull: " + spindexer.isFull + ", Str: " + spindexer.getArtifactString()
                    )

                    timer.reset()
                    if (spindexer.isEmpty) {
                        shootingState = Shoot.IDLE
                        spindexer.toFirstEmptyIntakePosition()
                    } else {
                        shootingState = Shoot.MOVE_SPINDEXER
                    }
                }
            }

            Shoot.IDLE                -> {
            }
        }
    }

    protected fun shootState(): State =
        InitializeState("Shoot state", { shootingState == Shoot.IDLE }, ::shootAllArtifacts)

    protected fun getMotifState(): State = InstantState("Motif", {
        spindexer.motifPattern = camera.motif
        motifPattern = spindexer.motifPattern
    })

    protected fun startIntake(): State = InstantState("Start intake", intake::intake)
}
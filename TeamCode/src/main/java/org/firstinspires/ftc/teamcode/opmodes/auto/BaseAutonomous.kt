package org.firstinspires.ftc.teamcode.opmodes.auto

import android.util.Log
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import com.skeletonarmy.marrow.prompts.OptionPrompt
import com.skeletonarmy.marrow.prompts.Prompter
import org.firstinspires.ftc.teamcode.opmodes.teleop.CombinedTeleOp
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
import org.firstinspires.ftc.teamcode.util.FullState
import org.firstinspires.ftc.teamcode.util.InitializeState
import org.firstinspires.ftc.teamcode.util.InstantState
import org.firstinspires.ftc.teamcode.util.State
import org.firstinspires.ftc.teamcode.util.TelemetryImplUpstreamSubmission
import org.firstinspires.ftc.teamcode.util.auto.BaseAutoPath
import org.firstinspires.ftc.teamcode.util.auto.Path

@Suppress("UNUSED")
abstract class BaseAutonomous: LinearOpMode() {
    companion object {
        const val PARAMETRIC_END = 0.94
        const val DEBUG_FSM = false
    }
    abstract fun initialize(alliance: Alliance)

    private var shootingState = Shoot.IDLE
    val timer = ElapsedTime()
    val matchTimer = ElapsedTime()
    val lastlastshottimer = ElapsedTime()

    enum class Shoot {
        IDLE,
        MOVE_SPINDEXER,
        WAIT_FOR_COMPLETION,
        WAIT_FOR_LAST_SHOT,
        LAST_SHOT_DELAY
    }

    fun shootAllArtifacts() {
        if (!spindexer.isEmpty) {
            shootingState = Shoot.MOVE_SPINDEXER
        }
    }

    fun updateShootingFSM() {
        when (shootingState) {
            Shoot.MOVE_SPINDEXER      -> {
                spindexer.toMotifOuttakePosition()
                if (DEBUG_FSM) Log.d("FSM", " * * * * * NEW CYCLE: * * * * * moving spindexer to ${spindexer.state.name}, ${spindexer.getArtifactString()}")
                timer.reset()
                shootingState = Shoot.WAIT_FOR_COMPLETION
            }

            Shoot.WAIT_FOR_COMPLETION -> {
                if (DEBUG_FSM) Log.d("FSM", "          Waiting for spindexer, current: ${spindexer.currentAngle}, target: ${spindexer.targetAngle}, diff: ${spindexer.targetAngle - spindexer.currentAngle}")
                if (spindexer.atSetPoint() && shooter.atSetPoint()) {
                    if (DEBUG_FSM) Log.d("FSM", "spindexer took ${timer.milliseconds()} to rotate")
                    transfer.on() // assume instantaneous transfer
                    spindexer.recordOuttake()

                    if (DEBUG_FSM) {
                        Log.d("FSM", "EVALUATING SPINDEXER FULLNESS")
                        Log.d("FSM", "Spindexer isEmpty: " + spindexer.isEmpty + ", isFull: " + spindexer.isFull + ", Str: " + spindexer.getArtifactString())
                    }

                    timer.reset()
                    shootingState = Shoot.WAIT_FOR_LAST_SHOT
                }
            }

            Shoot.WAIT_FOR_LAST_SHOT -> {
                // one loop between transfer on and go to next position, which is like 20-40ms
                shootingState = if (spindexer.isEmpty) {
                    lastlastshottimer.reset()
                    Shoot.LAST_SHOT_DELAY
                } else {
                    Shoot.MOVE_SPINDEXER
                }
            }

            Shoot.LAST_SHOT_DELAY -> {
                if (lastlastshottimer.milliseconds() in 50.0..150.0) {
                    transfer.reverse()
                } else if (lastlastshottimer.milliseconds() in 150.0..300.0) {
                    transfer.deReverse()
                } else if (lastlastshottimer.milliseconds() >= 300.0) {
                    endShootingCycle()
                }
            }

            Shoot.IDLE                -> { }
        }
    }

    fun endShootingCycle() {
        shootingState = Shoot.IDLE
        transfer.off()
        gamepad1.rumble(250)
        spindexer.toFirstEmptyIntakePosition()
    }

    val spindexer: Spindexer = Spindexer()
    val turret: Turret = Turret()
    val transfer: Transfer = Transfer()
    val shooter: Shooter = Shooter()
    val intake: Intake = Intake()
    val colorSensor: ColorSensor = ColorSensor()
    val camera: OV9281 = OV9281()

    val subsystems = setOf(spindexer, turret, transfer, shooter, intake, colorSensor, camera)

    protected lateinit var pathSequence: BaseAutoPath
    protected lateinit var finiteStateMachine: FiniteStateMachine
    protected lateinit var paths: List<Path>

    override fun runOpMode() {
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, TelemetryImplUpstreamSubmission(this))

        follower = Constants.createFollower(hardwareMap)

        subsystems.forEach { it.initialize() }

        val prompter = Prompter(this)

        prompter
            .prompt("alliance", OptionPrompt("Select Alliance", Alliance.RED, Alliance.BLUE))
            .onComplete {
                alliance = prompter.get<Alliance>("alliance")
                initialize(alliance!!)
                turret.selectedAlliance = alliance!!
                telemetry.addLine("six seven")
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
                turret.automatic = true
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

            follower!!.update()
            turret.robotPose = follower!!.pose
            shooter.setTargetState(turret.goalPose.distanceFrom(turret.turretPose))

            telemetry.addData("x", follower!!.pose.x)
            telemetry.addData("y", follower!!.pose.y)
            telemetry.addData("heading", follower!!.pose.heading)
            telemetry.addData("t", follower!!.pathCompletion)
            subsystems.forEach { it.periodic() }
            Drawing.drawDebug(follower!!)
            telemetry.update()
            BetterLoopTimeComponent.postUpdate()
        }

        follower!!.breakFollowing()
    }

//todo
    protected fun shootState(): State =
        InitializeState("Shoot state", { shootingState == Shoot.IDLE || shootingState == Shoot.LAST_SHOT_DELAY && lastlastshottimer.milliseconds() >= 200.0 }, ::shootAllArtifacts)

//    protected fun shootState() = WaitState(700.0)

    protected fun startIntake(): State = InstantState("Start intake", intake::intake)
    protected fun stopIntake(): State = InstantState("Stop Intake", intake::off)

//    protected  fun startIntake(): State = WaitState(250.0)

    val poses = mutableListOf<Pose>()
    var doneRelo = false
    protected fun relocalizeState(): State = FullState("Relo", { doneRelo }, { doneRelo = false }, {
        if (camera.turretPose != Pose(0.0,0.0) )poses.add(camera.turretPose)

        if (poses.size >= 30) {
            follower!!.pose = averageRelocalization()
            doneRelo = true
        }
    })

    fun averageRelocalization(): Pose {
        var totalX = 0.0
        var totalY = 0.0
        poses.forEach {
            pose -> totalX += pose.x
            totalY += pose.y
        }

        return if (totalX >= 1.0 && totalY >= 1.0) {
            Pose(
                totalX / poses.size,
                totalY / poses.size,
                follower!!.heading
            )
        } else {
            follower!!.pose
        }
    }
}
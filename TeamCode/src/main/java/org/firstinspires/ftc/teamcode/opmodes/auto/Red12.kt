package org.firstinspires.ftc.teamcode.opmodes.auto

import android.util.Log
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathBuilder
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
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
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.Artifact
import org.firstinspires.ftc.teamcode.util.BetterLoopTimeComponent
import org.firstinspires.ftc.teamcode.util.FiniteStateMachine
import org.firstinspires.ftc.teamcode.util.InitConfigurer
import org.firstinspires.ftc.teamcode.util.buildBasicLine
import org.firstinspires.ftc.teamcode.util.buildCurvedLine
import org.firstinspires.ftc.teamcode.util.buildTangentLine

@Suppress("UNUSED")
@Autonomous(name = "12 ball red near", preselectTeleOp = "Combined TeleOp")
class Red12 : LinearOpMode() {
    enum class Shoot {
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
    val colorSensor: ArtifactColorSensor = ArtifactColorSensor()
    val camera: OV9281 = OV9281()

    val subsystems = setOf(spindexer, turret, transfer, shooter, intake, colorSensor)

    val finiteStateMachine: FiniteStateMachine = FiniteStateMachine()

    override fun runOpMode() {
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)

        follower = Constants.createFollower(hardwareMap)
        buildPaths()
        follower!!.setStartingPose(nearStartPose)
        follower!!.update()
        Drawing.init()
        Drawing.drawDebug(follower)
        InitConfigurer.selectedAlliance = Alliance.RED

        subsystems.forEach { it.initialize() }
        camera.initialize()
        turret.selectedAlliance = Alliance.RED

        Log.d("FSM", "auto inited")
        while (opModeInInit()) {
            follower!!.update()
            telemetry.update()
        }

        BetterLoopTimeComponent.preStartButtonPressed()
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
                    follower!!.followPath(scorePreload, 1.0, true)
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
                {follower!!.followPath(dIntake1, 0.8, true)}
            ).addState(
                "Intake 1",
                {!follower!!.isBusy},
                {follower!!.followPath(intake1, 0.4, true)}
            ).addState(
                "DScore 1",
                {!follower!!.isBusy},
                {follower!!.followPath(score1, 1.0, true)}
            ).addState(
                "Shoot 1",
                {!follower!!.isBusy},
                ::shootAllArtifacts
            ).addState(
                "DIntake 2",
                {shootingState == Shoot.IDLE},
                {follower!!.followPath(dIntake2, 0.8, true)}
            ).addState(
                "Intake 2",
                {!follower!!.isBusy},
                {follower!!.followPath(intake2, 0.4, true)}
            ).addState(
                "DScore 2",
                {!follower!!.isBusy},
                {follower!!.followPath(score2, 1.0, true)}
            ).addState(
                "Shoot 2",
                {!follower!!.isBusy},
                ::shootAllArtifacts
            ).addState(
                "DIntake 3",
                {shootingState == Shoot.IDLE},
                {follower!!.followPath(dIntake3, 0.8, true)}
            ).addState(
                "Intake 3",
                {!follower!!.isBusy},
                {follower!!.followPath(intake3, 0.4, true)}
            ).addState(
                "DScore 3",
                {!follower!!.isBusy},
                {follower!!.followPath(score3, 1.0, true)}
            ).addState(
                "Shoot 3",
                {!follower!!.isBusy},
                ::shootAllArtifacts
            ).addState(
                "Park",
                {shootingState == Shoot.IDLE},
                {follower!!.followPath(park, 1.0, true)}
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
            BetterLoopTimeComponent.postUpdate()
        }

        follower!!.breakFollowing()
    }

    lateinit var scorePreload: PathChain
    lateinit var dIntake1: PathChain
    lateinit var intake1: PathChain
    lateinit var score1: PathChain
    lateinit var dIntake2: PathChain
    lateinit var intake2: PathChain
    lateinit var score2: PathChain
    lateinit var dIntake3: PathChain
    lateinit var intake3: PathChain
    lateinit var score3: PathChain
    lateinit var park: PathChain

    fun buildPaths() {
        scorePreload = follower!!.pathBuilder()
            .buildBasicLine(nearStartPose, nearShootPose).build()

        dIntake1 = follower!!.pathBuilder()
            .buildCurvedLine(nearShootPose, intake1Control, startIntake1).build()

        intake1 = follower!!.pathBuilder()
            .buildTangentLine(startIntake1, endIntake1).build()

        score1 = follower!!.pathBuilder()
            .buildBasicLine(endIntake1, nearShootPose).build()

        dIntake2 = follower!!.pathBuilder()
            .buildCurvedLine(nearShootPose, intake2Control, startIntake2).build()

        intake2 = follower!!.pathBuilder()
            .buildTangentLine(startIntake2, endIntake2).build()

        score2 = follower!!.pathBuilder()
            .buildTangentLine(endIntake2, endIntake2Move).setReversed()
            .buildBasicLine(endIntake2Move, nearShootPose).build()

        dIntake3 = follower!!.pathBuilder()
            .buildCurvedLine(nearShootPose, intake3Control, startIntake3).build()

        intake3 = follower!!.pathBuilder()
            .buildTangentLine(startIntake3, endIntake3).build()

        score3 = follower!!.pathBuilder()
            .buildTangentLine(endIntake3, endIntake3Move).setReversed()
            .buildBasicLine(endIntake3, nearShootPose).build()

        park = follower!!.pathBuilder()
            .buildBasicLine(nearShootPose, nearParkPose).build()
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
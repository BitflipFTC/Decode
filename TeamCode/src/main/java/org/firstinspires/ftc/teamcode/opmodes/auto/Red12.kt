package org.firstinspires.ftc.teamcode.opmodes.auto

import android.util.Log
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
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
import org.firstinspires.ftc.teamcode.util.MotifPattern

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
                    intake.off()
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
                "Intake 1",
                {shootingState == Shoot.IDLE},
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
                "Intake 2",
                {shootingState == Shoot.IDLE},
                {follower!!.followPath(intake2, 0.4, true)}
            ).addState(
                "DScore 2",
                {!follower!!.isBusy},
                {follower!!.followPath(score2, 1.0, true)}
            ).addState(
                "Shoot 2",
                {!follower!!.isBusy},
                ::shootAllArtifacts
//            ).addState(
//                "DIntake 3",
//                {shootingState == Shoot.IDLE},
//                {follower!!.followPath(intake3, 0.7, true)}
//            ).addState(
//                "DScore 3",
//                {!follower!!.isBusy},
//                {follower!!.followPath(score3, 1.0, true)}
//            ).addState(
//                "Shoot 3",
//                {!follower!!.isBusy},
//                ::shootAllArtifacts
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
    lateinit var intake1: PathChain
    lateinit var score1: PathChain
    lateinit var intake2: PathChain
    lateinit var score2: PathChain
    lateinit var intake3: PathChain
    lateinit var score3: PathChain
    lateinit var park: PathChain

    fun buildPaths() {
        scorePreload = follower!!.pathBuilder()
            .addPath(
                BezierLine(
                    nearStartPose,
                    nearShootPose
                )
            )
            .setLinearHeadingInterpolation(nearStartPose.heading, nearShootPose.heading)
            .addParametricCallback(0.0) { intake.intake() }
//            .addParametricCallback(1.0) {
//                spindexer.motifPattern = camera.motif
//                CombinedTeleOp.motifPattern = spindexer.motifPattern
//                Log.d("FSM", "motif pattern detected: ${spindexer.motifPattern}")
//            }
            .build()

        intake1 = follower!!.pathBuilder()
            .addPath(
                BezierCurve(
                    nearShootPose,
                    intake1Control,
                    startIntake1
                )
            )
            .setLinearHeadingInterpolation(nearShootPose.heading, startIntake1.heading)
            .addPath(
                BezierLine(startIntake1, endIntake1)
            )
            .setTangentHeadingInterpolation()
            .build()

        score1 = buildBasicLine(endIntake1, nearShootPose)

        intake2 = follower!!.pathBuilder()
            .addPath(
                BezierCurve(
                    nearShootPose,
                    intake2Control,
                    startIntake2
                )
            )
            .setLinearHeadingInterpolation(nearShootPose.heading, startIntake2.heading)
            .addPath(
                BezierLine(startIntake2, endIntake2)
            )
            .setTangentHeadingInterpolation()
            .addPath(
                BezierLine(
                    endIntake2,
                    endIntake2Move
                )
            )
            .setTangentHeadingInterpolation().setReversed()
            .build()

        score2 = buildBasicLine(endIntake2Move, nearShootPose)

        intake3 = follower!!.pathBuilder()
            .addPath(
                BezierCurve(
                    nearShootPose,
                    intake3Control,
                    startIntake3
                )
            )
            .setLinearHeadingInterpolation(nearShootPose.heading, startIntake3.heading)
            .addPath(
                BezierLine(startIntake3, endIntake3)
            )
            .setTangentHeadingInterpolation()
            .addPath(
                BezierLine(endIntake3, endIntake3Move)
            )
            .setTangentHeadingInterpolation().setReversed()
            .build()

        score3 = buildBasicLine(endIntake3, nearShootPose)

        park = buildBasicLine(nearShootPose, nearParkPose)
    }

    fun buildBasicLine(p1: Pose, p2: Pose): PathChain = follower!!.pathBuilder()
        .addPath(BezierLine(p1, p2))
        .setLinearHeadingInterpolation(p1.heading, p2.heading)
        .build()

    fun buildTangentLine(p1: Pose, p2: Pose): PathChain = follower!!.pathBuilder()
        .addPath(BezierLine(p1, p2))
        .setTangentHeadingInterpolation()
        .build()

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
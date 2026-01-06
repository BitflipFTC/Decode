package org.firstinspires.ftc.teamcode.opmodes.auto

import android.service.credentials.CallingAppInfo
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

@Suppress("UNUSED")
@Autonomous(name = "12 ball blue near", preselectTeleOp = "Combined TeleOp")
class Blue12 : LinearOpMode() {
    enum class PathState {
        Start,
        DScorePreload,
        ScorePreload,
        DIntake1,
        DScore1,
        Score1,
        DIntake2,
        DScore2,
        Score2,
        DIntake3,
        DScore3,
        Score3,
        DPark
    }

    var pathState: PathState = PathState.Start

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

    override fun runOpMode() {
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)

        follower = Constants.createFollower(hardwareMap)
        buildPaths()
        follower!!.setStartingPose(nearStartPose.mirror())
        follower!!.update()
        Drawing.init()
        Drawing.drawDebug(follower)

        subsystems.forEach { it.initialize() }
        camera.initialize()
        turret.selectedAlliance = Alliance.BLUE

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
        pathState = PathState.DScorePreload

        while (opModeIsActive()) {
            autonomousPathUpdate()
            updateShootingFSM()

            val artifactDetected =
                colorSensor.detectedArtifact != null && !spindexer.isFull && spindexer.slotsToIntakes.contains(
                    spindexer.state
                ) && spindexer.atSetPoint()
            if (artifactDetected) {
                spindexer.recordIntake(colorSensor.detectedArtifact!!)
                spindexer.toFirstEmptyIntakePosition()
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
                    nearStartPose.mirror(),
                    nearShootPose.mirror()
                )
            )
            .setLinearHeadingInterpolation(nearStartPose.mirror().heading, Math.toRadians(90.0))
            .addParametricCallback(0.0) { intake.intake() }
            .addParametricCallback(1.0) {
                spindexer.motifPattern = camera.motif
                CombinedTeleOp.motifPattern = spindexer.motifPattern
            }
            .build()

        intake1 = follower!!.pathBuilder()
            .addPath(
                BezierCurve(
                    nearShootPose.mirror(),
                    intake1Control.mirror(),
                    startIntake1.mirror()
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(90.0), startIntake1.mirror().heading)
            .addPath(
                BezierLine(startIntake1.mirror(), endIntake1.mirror())
            )
            .setTangentHeadingInterpolation()
            .build()

        score1 = buildBasicLine(endIntake1.mirror(), nearShootPose.mirror())

        intake2 = follower!!.pathBuilder()
            .addPath(
                BezierCurve(
                    nearShootPose.mirror(),
                    intake2Control.mirror(),
                    startIntake2.mirror()
                )
            )
            .setLinearHeadingInterpolation(nearShootPose.mirror().heading, startIntake2.mirror().heading)
            .addPath(
                BezierLine(startIntake2.mirror(), endIntake2.mirror())
            )
            .setTangentHeadingInterpolation()
            .addPath(
                BezierLine(
                    endIntake2.mirror(),
                    endIntake2Move.mirror()
                )
            )
            .setTangentHeadingInterpolation().setReversed()
            .build()

        score2 = buildBasicLine(endIntake2Move.mirror(), nearShootPose.mirror())

        intake3 = follower!!.pathBuilder()
            .addPath(
                BezierCurve(
                    nearShootPose.mirror(),
                    intake3Control.mirror(),
                    startIntake3.mirror()
                )
            )
            .setLinearHeadingInterpolation(nearShootPose.mirror().heading, startIntake3.mirror().heading)
            .addPath(
                BezierLine(startIntake3.mirror(), endIntake3.mirror())
            )
            .setTangentHeadingInterpolation()
            .addPath(
                BezierLine(endIntake3.mirror(), endIntake3Move.mirror())
            )
            .setTangentHeadingInterpolation().setReversed()
            .build()

        score3 = buildBasicLine(endIntake3.mirror(), nearShootPose.mirror())

        park = buildBasicLine(nearShootPose.mirror(), nearParkPose.mirror())
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
    fun autonomousPathUpdate() {
        when (pathState) {
            PathState.Start         -> {} // do nothing
            PathState.DScorePreload -> {
                follower!!.followPath(scorePreload, 1.0, true)
                intake.off()
                pathState = PathState.ScorePreload
            }

            PathState.ScorePreload  -> {
                if (!follower!!.isBusy) {
                    shootAllArtifacts()
                    pathState = PathState.DIntake1
                }
            }

            PathState.DIntake1      -> {
                if (shootingState == Shoot.IDLE) {
                    follower!!.followPath(intake1, 0.7, true)
                    pathState = PathState.DScore1
                }
            }

            PathState.DScore1       -> {
                if (!follower!!.isBusy) {
                    follower!!.followPath(score1, 1.0, true)
                    pathState = PathState.Score1
                }
            }

            PathState.Score1        -> {
                if (!follower!!.isBusy) {
                    shootAllArtifacts()
                    pathState = PathState.DIntake2
                }
            }

            PathState.DIntake2      -> {
                if (shootingState == Shoot.IDLE) {
                    follower!!.followPath(intake2, 0.7, true)
                    pathState = PathState.DScore2
                }
            }

            PathState.DScore2       -> {
                if (!follower!!.isBusy) {
                    follower!!.followPath(score2, 1.0, true)
                    pathState = PathState.Score2
                }
            }

            PathState.Score2        -> {
                if (!follower!!.isBusy) {
                    shootAllArtifacts()
                    pathState = PathState.DIntake3
                }
            }

            PathState.DIntake3      -> {
                if (shootingState == Shoot.IDLE) {
                    follower!!.followPath(intake3, 0.7, true)
                    pathState = PathState.DScore3
                }
            }

            PathState.DScore3       -> {
                if (!follower!!.isBusy) {
                    follower!!.followPath(score3, 1.0, true)
                    pathState = PathState.Score3
                }
            }

            PathState.Score3        -> {
                if (!follower!!.isBusy) {
                    shootAllArtifacts()
                    pathState = PathState.DPark
                    timer.reset()
                }
            }

            PathState.DPark         -> {
                if (shootingState == Shoot.IDLE || timer.seconds() > 2.5) {
                    follower!!.followPath(park, 1.0, true)
                }
            }
        }
    }

    fun shootAllArtifacts() {
        if (!spindexer.isEmpty) {
            shootingState = Shoot.MOVE_SPINDEXER
        }
    }

    fun updateShootingFSM() {
        when (shootingState) {
            Shoot.MOVE_SPINDEXER      -> {
                spindexer.toFirstFullOuttakePosition()
                Log.d(
                    "FSM",
                    "MOVING SPINDEXER TO ${spindexer.state.name}, ${spindexer.getArtifactString()}"
                )
                shootingState = Shoot.TRANSFER_ARTIFACT
            }

            Shoot.TRANSFER_ARTIFACT   -> {
                Log.d("FSM", "Waiting for shooter or spindexer")
                Log.d(
                    "FSM",
                    "sp: ${spindexer.currentAngle}, ${spindexer.targetAngle}, sh: ${shooter.flywheelRPM}, ${shooter.targetFlywheelRPM}"
                )
                if (shooter.atSetPoint() && spindexer.atSetPoint()) {
                    transfer.transferArtifact()
                    Log.d("FSM", "TRANSFERING")
                    shootingState = Shoot.WAIT_FOR_COMPLETION
                }
            }

            Shoot.WAIT_FOR_COMPLETION -> {
                Log.d(
                    "FSM",
                    "WAITING FOR TRANSFER, ${transfer.currentPosition}, ${transfer.targetPosition}"
                )
                if (transfer.atSetPoint()) {
                    spindexer.recordOuttake()
                    Log.d("FSM", "EVALUATING SPINDEXER FULLNESS")
                    Log.d(
                        "FSM",
                        "Spindexer isEmpty: " + spindexer.isEmpty + ", isFull: " + spindexer.isFull + ", Str: " + spindexer.getArtifactString()
                    )

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
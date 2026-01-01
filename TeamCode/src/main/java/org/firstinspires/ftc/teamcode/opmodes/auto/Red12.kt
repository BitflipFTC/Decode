package org.firstinspires.ftc.teamcode.opmodes.auto

import android.util.Log
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing
import org.firstinspires.ftc.teamcode.subsystems.ArtifactColorSensor
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Spindexer
import org.firstinspires.ftc.teamcode.subsystems.Transfer
import org.firstinspires.ftc.teamcode.subsystems.Turret
import org.firstinspires.ftc.teamcode.util.Artifact
import org.firstinspires.ftc.teamcode.util.BetterLoopTimeComponent

@Suppress("UNUSED")
@Autonomous(name="12 ball red")
class Red12: LinearOpMode() {
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

    lateinit var follower: Follower
    val spindexer: Spindexer = Spindexer()
    val turret: Turret = Turret()
    val transfer: Transfer = Transfer()
    val shooter: Shooter = Shooter()
    val intake: Intake = Intake()
    val colorSensor: ArtifactColorSensor = ArtifactColorSensor()

    val subsystems = setOf(spindexer, turret, transfer, shooter, intake, colorSensor)

    override fun runOpMode() {
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)

        follower = Constants.createFollower(hardwareMap)
        buildPaths()
        follower.setStartingPose(nearStartPose)
        follower.update()
        Drawing.init()
        Drawing.drawDebug(follower)

        subsystems.forEach { it.initialize() }

        while (opModeInInit()) {
            follower.update()
            telemetry.update()
        }

        BetterLoopTimeComponent.preStartButtonPressed()
        // set up preloads
        spindexer.setCollectedArtifacts(
            Artifact.PURPLE,
            Artifact.PURPLE,
            Artifact.GREEN
        )
        pathState = PathState.DScorePreload

        while (opModeIsActive()) {
            autonomousPathUpdate()
            updateShootingFSM()

            val artifactDetected = colorSensor.detectedArtifact != null && !spindexer.isFull && spindexer.slotsToIntakes.contains(spindexer.state) && spindexer.atSetPoint()
            if (artifactDetected) {
                spindexer.recordIntake(colorSensor.detectedArtifact!!)
                spindexer.toFirstEmptyIntakePosition()
            }

            shooter.hoodPosition = 0.1
            shooter.targetFlywheelRPM = 3500.0
//            shooter.setTargetState(turret.goalPose.distanceFrom(follower.pose))
            turret.robotPose = follower.pose

            follower.update()
            telemetry.addData("x", follower.pose.x)
            telemetry.addData("y", follower.pose.y)
            telemetry.addData("heading", follower.pose.heading)
            subsystems.forEach { it.periodic() }
            Drawing.drawDebug(follower)
            telemetry.update()
            BetterLoopTimeComponent.postUpdate()
        }
    }

    val horizontalIntakeStart = 100.0
    val horizontalIntakeEnd   = 125.0

    val intake1Vertical = 83.0
    val intake2Vertical = 58.5
    val intake3Vertical = 34.0
    val intakeHeading = Math.toRadians(0.0)

    val farStartPose     = Pose(88.0,  8.5,    Math.toRadians(90.0))
    val farShootPose     = Pose(88.0,  14.0,   Math.toRadians(0.0))
    val farParkPose      = Pose(105.500,  33.500, Math.toRadians(90.0))
    val nearStartPose    = Pose(121.0, 125.0,  Math.toRadians(35.954))
    val nearShootPose    = Pose(96.0,  96.0,   Math.toRadians(0.0))
    val nearParkPose     = Pose(125.0, 90.0, Math.toRadians(0.0))
    val startIntake1  = Pose(horizontalIntakeStart, intake1Vertical, intakeHeading)
    val endIntake1    = Pose(horizontalIntakeEnd,   intake1Vertical, intakeHeading)
    val startIntake2  = Pose(horizontalIntakeStart, intake2Vertical, intakeHeading)
    val endIntake2    = Pose(horizontalIntakeEnd,   intake2Vertical, intakeHeading)
    val startIntake3  = Pose(horizontalIntakeStart, intake3Vertical, intakeHeading)
    val endIntake3    = Pose(horizontalIntakeEnd,   intake3Vertical, intakeHeading)

    val emptyRampControl = Pose(horizontalIntakeEnd, 74.0, Math.toRadians(90.0))
    val emptyRamp = Pose(128.0, 74.0, Math.toRadians(90.0))

    lateinit var scorePreload: PathChain
    lateinit var intake1: PathChain
    lateinit var score1: PathChain
    lateinit var intake2: PathChain
    lateinit var score2: PathChain
    lateinit var intake3: PathChain
    lateinit var score3: PathChain
    lateinit var park: PathChain

    fun buildPaths() {
        scorePreload = buildBasicLine(nearStartPose, nearShootPose)
        intake1 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    nearShootPose,
                    Pose(nearStartPose.x, intake1Vertical),
                    startIntake1
                )
            )
            .setLinearHeadingInterpolation(nearShootPose.heading, startIntake1.heading)
            .addPath(
                BezierLine(startIntake1, endIntake1)
            )
            .setTangentHeadingInterpolation()
            .addPath(
                BezierCurve(
                    endIntake1,
                    emptyRampControl,
                    emptyRamp
                )
            )
            .setConstantHeadingInterpolation(emptyRamp.heading)
            .build()

        // spin spindexer

        score1 = buildBasicLine(emptyRamp, nearShootPose)
        // shoot + aim

        intake2 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    nearShootPose,
                    Pose(nearShootPose.x, intake2Vertical),
                    startIntake2
                )
            )
            .setLinearHeadingInterpolation(nearShootPose.heading, startIntake2.heading)
            .addPath(
                BezierLine(startIntake2, endIntake2)
            )
            .setTangentHeadingInterpolation()
            .build()

        score2 = buildBasicLine(endIntake2,nearShootPose)

        intake3 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    nearShootPose,
                    Pose(nearShootPose.x, intake3Vertical),
                    startIntake3
                )
            )
            .setLinearHeadingInterpolation(nearShootPose.heading, startIntake3.heading)
            .addPath(
                BezierLine(startIntake3, endIntake3)
            )
            .setTangentHeadingInterpolation()
            .build()

        score3 = buildBasicLine(endIntake3,farShootPose)

        park = buildTangentLine(farShootPose, farParkPose)
    }

    fun buildBasicLine(p1: Pose, p2: Pose): PathChain = follower.pathBuilder()
        .addPath(BezierLine(p1, p2))
        .setLinearHeadingInterpolation(p1.heading,p2.heading)
        .build()

    fun buildTangentLine(p1: Pose, p2: Pose): PathChain = follower.pathBuilder()
        .addPath(BezierLine(p1, p2))
        .setTangentHeadingInterpolation()
        .build()

    fun autonomousPathUpdate () {
        when (pathState) {
            PathState.Start         -> {} // do nothing
            PathState.DScorePreload -> {
                follower.followPath(scorePreload, 0.75, true)
                intake.off()
                pathState = PathState.ScorePreload
            }
            PathState.ScorePreload  -> {
                if (!follower.isBusy) {
                    shootAllArtifacts()
                    pathState = PathState.DIntake1
                }
            }
            PathState.DIntake1      -> {
                if (shootingState == Shoot.IDLE) {
                    intake.intake()
                    follower.followPath(intake1, 0.75, true)
                    pathState = PathState.DScore1
                }
            }
            PathState.DScore1       -> {
                if (!follower.isBusy) {
                    intake.off()
                    follower.followPath(score1, 0.75, true)
                    pathState = PathState.Score1
                }
            }
            PathState.Score1        -> {
                if (!follower.isBusy) {
                    shootAllArtifacts()
                    pathState = PathState.DIntake2
                }
            }
            PathState.DIntake2      -> {
                if (shootingState == Shoot.IDLE) {
                    intake.intake()
                    follower.followPath(intake2, 0.75, true)
                    pathState = PathState.DScore2
                }
            }
            PathState.DScore2       -> {
                if (!follower.isBusy) {
                    intake.off()
                    follower.followPath(score2, 0.75, true)
                    pathState = PathState.Score2
                }
            }
            PathState.Score2        -> {
                if (!follower.isBusy) {
                    shootAllArtifacts()
                    pathState = PathState.DIntake3
                }
            }
            PathState.DIntake3      -> {
                if (shootingState == Shoot.IDLE) {
                    intake.intake()
                    follower.followPath(intake3, 0.75, true)
                    pathState = PathState.DScore3
                }
            }
            PathState.DScore3       -> {
                if (!follower.isBusy) {
                    intake.off()
                    follower.followPath(score3, 0.75, true)
                    pathState = PathState.Score3
                }
            }
            PathState.Score3        -> {
                if (!follower.isBusy) {
                    shootAllArtifacts()
                    pathState = PathState.DPark
                }
            }
            PathState.DPark         -> {
                if (shootingState == Shoot.IDLE) {
                    follower.followPath(park, 0.75, true)
                }
            }
        }
    }

    fun shootAllArtifacts() {
        if (!spindexer.isEmpty) {
            shootingState = Shoot.MOVE_SPINDEXER
        }
    }

    fun updateShootingFSM () {
        when (shootingState) {
            Shoot.MOVE_SPINDEXER      -> {
                spindexer.toMotifOuttakePosition()
                Log.d("FSM", "MOVING SPINDEXER TO ${spindexer.state.name}, ${spindexer.getArtifactString()}")
                shootingState = Shoot.TRANSFER_ARTIFACT
            }
            Shoot.TRANSFER_ARTIFACT   -> {
                Log.d("FSM", "Waiting for shooter or spindexer")
                Log.d("FSM", "sp: ${spindexer.currentAngle}, ${spindexer.targetAngle}, sh: ${shooter.flywheelRPM}, ${shooter.targetFlywheelRPM}")
                if (shooter.atSetPoint() && spindexer.atSetPoint()) {
                    transfer.transferArtifact()
                    Log.d("FSM", "TRANSFERING")
                    shootingState = Shoot.WAIT_FOR_COMPLETION
                }
            }
            Shoot.WAIT_FOR_COMPLETION -> {
                Log.d("FSM", "WAITING FOR TRANSFER, ${transfer.currentPosition}, ${transfer.targetPosition}")
                if (transfer.atSetPoint()) {
                    spindexer.recordOuttake()
                    Log.d("FSM", "EVALUATING SPINDEXER FULLNESS")
                    Log.d("FSM", "Spindexer isEmpty: " + spindexer.isEmpty + ", isFull: " + spindexer.isFull + ", Str: " + spindexer.getArtifactString())

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
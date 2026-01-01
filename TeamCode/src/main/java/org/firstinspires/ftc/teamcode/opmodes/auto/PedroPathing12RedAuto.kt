package org.firstinspires.ftc.teamcode.opmodes.auto

import android.util.Log
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.bindings.button
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.m
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing
import org.firstinspires.ftc.teamcode.util.Artifact
import org.firstinspires.ftc.teamcode.util.BetterLoopTimeComponent
import org.firstinspires.ftc.teamcode.util.BitflipOpMode
import org.firstinspires.ftc.teamcode.util.InitConfigurer
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

@Suppress("UNUSED")
//@Autonomous(name = "pp 12")
class PedroPathing12RedAuto: BitflipOpMode() {
    init {
        addComponents(
            SubsystemComponent(
                spindexer,
                intake,
                turret,
                camera,
                shooter,
                transfer,
                colorSensor
            ),
            PedroComponent(Constants::createFollower),
            InitConfigurer,
            BetterLoopTimeComponent
        )
    }

    fun drawOnlyCurrent() {
        try {
            Drawing.drawRobot(PedroComponent.follower.pose)
            Drawing.sendPacket()
        } catch (e: Exception) {
            throw RuntimeException("Drawing failed $e")
        }
    }

    fun draw() {
        Drawing.drawDebug(PedroComponent.follower)
    }

    override fun onInit() {
        buildPaths()
        PedroComponent.follower.setStartingPose(nearStartPose)
        drawOnlyCurrent()
        Drawing.init()
    }

    override fun waitForStart() {
        drawOnlyCurrent()
        Drawing.sendPacket()
    }

    override fun onStartButtonPressed() {
        camera.targetID = InitConfigurer.selectedAlliance?.aprilTagID ?: 24
        spindexer.motifPattern = camera.getMotif()

        val autoaim = LambdaCommand()
            .setUpdate {
                turret.robotPose = PedroComponent.follower.pose
            }
            .setIsDone { false }
            .setRequirements(turret)
            .setName("Autoaim")
            .setInterruptible(false)

        val autoadjust = LambdaCommand()
            .setUpdate {
                shooter.targetFlywheelRPM = 3000.0
                shooter.hoodPosition = 0.05
            }
            .setIsDone { false }
            .setRequirements(shooter)
            .setName("AutoAdjust")
            .setInterruptible(false)

        val setUpPreloads = SequentialGroup(
            InstantCommand {
                spindexer.setCollectedArtifacts(
                    Artifact.PURPLE,
                    Artifact.PURPLE,
                    Artifact.GREEN
                )
            }
        )


        autoadjust()
        autoaim()
        setUpPreloads()
        intake.runIntake()()
//        spindexer.goToMotifOuttake()()

//        button {colorSensor.detectedArtifact != null && !spindexer.isFull && spindexer.slotsToIntakes.contains(spindexer.state) && spindexer.atSetPoint()}.whenBecomesTrue {
//            SequentialGroup (
//                InstantCommand { spindexer.recordIntake(colorSensor.detectedArtifact!!) },
//                spindexer.goToFirstEmptyIntake()
//            )
//        }

        val autonomousRoutine = SequentialGroup(
            FollowPath(scorePreload, holdEnd = true, maxPower = 0.5),
            shootAllArtifacts(),
            FollowPath(intake1, holdEnd = true, maxPower = 0.5),
            FollowPath(score1),
            shootAllArtifacts(),
            FollowPath(intake2, holdEnd = true, maxPower = 0.75),
            FollowPath(score2),
            shootAllArtifacts(),
            FollowPath(intake3, holdEnd = true, maxPower = 0.75),
            FollowPath(score3, holdEnd = true, maxPower = 0.75),
            shootAllArtifacts(),
            FollowPath(park),
        )

        autonomousRoutine()
    }

    override fun onUpdate() {
        telemetry.addData("x", PedroComponent.follower.pose.x)
        telemetry.addData("y", PedroComponent.follower.pose.y)
        telemetry.addData("heading", PedroComponent.follower.pose.heading)
        draw()

        Log.d("ROBOT", "SPINDEXER AT ${spindexer.state.name}")
        Log.d("ROBOT", "transfer at ${transfer.currentPosition}")
        Log.d("ROBOT", CommandManager.snapshot.toString())
    }

    val horizontalIntakeStart = 100.0
    val horizontalIntakeEnd   = 125.0

    val intake1Vertical = 34.0
    val intake2Vertical = 58.5
    val intake3Vertical = 83.0

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
        intake1 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    nearShootPose,
                    Pose(nearStartPose.x, intake3Vertical),
                    startIntake3
                )
            )
            .setLinearHeadingInterpolation(nearShootPose.heading, startIntake3.heading)
            .addPath(
                BezierLine(startIntake3, endIntake3)
            )
            .setTangentHeadingInterpolation()
            .addPath(
                BezierCurve(
                    endIntake3,
                    emptyRampControl,
                    emptyRamp
                )
            )
            .setConstantHeadingInterpolation(emptyRamp.heading)
            .build()
        
        // spin spindexer

        score1 = buildBasicLine(emptyRamp, nearShootPose)
        // shoot + aim

        intake2 = PedroComponent.follower.pathBuilder()
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

        intake3 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    nearShootPose,
                    Pose(nearShootPose.x, intake1Vertical),
                    startIntake1
                )
            )
            .setLinearHeadingInterpolation(nearShootPose.heading, startIntake1.heading)
            .addPath(
                BezierLine(startIntake1, endIntake1)
            )
            .setTangentHeadingInterpolation()
            .build()

        score3 = buildBasicLine(endIntake3,farShootPose)

        park = buildTangentLine(farShootPose, farParkPose)
    }

    fun buildBasicLine(p1: Pose, p2: Pose): PathChain = PedroComponent.follower.pathBuilder()
        .addPath(BezierLine(p1, p2))
        .setLinearHeadingInterpolation(p1.heading,p2.heading)
        .build()

    fun buildTangentLine(p1: Pose, p2: Pose): PathChain = PedroComponent.follower.pathBuilder()
        .addPath(BezierLine(p1, p2))
        .setTangentHeadingInterpolation()
        .build()
}
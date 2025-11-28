package org.firstinspires.ftc.teamcode.opmodes.auto

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing
import org.firstinspires.ftc.teamcode.util.Artifact
import org.firstinspires.ftc.teamcode.util.BitflipOpMode
import org.firstinspires.ftc.teamcode.util.InitConfigurer
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

@Autonomous(name = "pp 12")
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
            PedroComponent(
                Constants::createFollower
            ),
            InitConfigurer
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
        PedroComponent.follower.setStartingPose(startPose)
    }

    override fun waitForStart() {
        drawOnlyCurrent()
    }

    val waitUntilReadyToShoot: ParallelGroup
        get() = ParallelGroup(
            WaitUntil(turret::atSetPoint),
            WaitUntil(shooter::atSetPoint)
        )

    override fun onStartButtonPressed() {
        camera.targetID = InitConfigurer.selectedAlliance.aprilTagID
        spindexer.motifPattern = camera.getMotif()

        val autoaim = LambdaCommand()
            .setUpdate {
                turret.bearing = camera.currentTagBearing
            }
            .setIsDone { false }
            .setRequirements(turret)
            .setName("Autoaim")
            .setInterruptible(false)

        val autoadjust = LambdaCommand()
            .setUpdate {
                shooter.setTargetState(camera.distanceToGoal)
            }
            .setIsDone { false }
            .setRequirements(shooter)
            .setName("AutoAdjust")
            .setInterruptible(false)

        val setUpPreloads = SequentialGroup(
            InstantCommand { spindexer.recordIntake(Artifact.PURPLE, 0) },
            InstantCommand { spindexer.recordIntake(Artifact.PURPLE, 1) },
            InstantCommand { spindexer.recordIntake(Artifact.GREEN,  2) }
        )

        val activateColorSensorDetection = LambdaCommand()
            .setUpdate {
                if (colorSensor.distance <= 5) {
                    spindexer.recordIntake(colorSensor.detectedArtifact)
                    spindexer.goToFirstEmptyIntake()()
                }
            }
            .setIsDone { false }
            .setRequirements(colorSensor)
            .setName("Color Sensor Detecting")
            .setInterruptible(true)

        autoadjust()
        autoaim()
        setUpPreloads()
        intake.runIntake()()

        val autonomousRoutine = SequentialGroup(
            FollowPath(scorePreload, holdEnd = true),
            Delay(2.seconds),
//            waitUntilReadyToShoot,
//            shootAllArtifacts(200.milliseconds),
//            activateColorSensorDetection,
            FollowPath(intake1, holdEnd = true),
            Delay(500.milliseconds),
//            InstantCommand { activateColorSensorDetection.cancel() },
            FollowPath(score1),
            Delay(2.seconds),
//            waitUntilReadyToShoot,
//            shootAllArtifacts(200.milliseconds),
//            activateColorSensorDetection,
            FollowPath(intake2, holdEnd = true),
            Delay(500.milliseconds),
//            InstantCommand { activateColorSensorDetection.cancel() },
            FollowPath(score2),
            Delay(2.seconds),
//            shootAllArtifacts(200.milliseconds),
//            activateColorSensorDetection,
            FollowPath(intake3, holdEnd = true),
            Delay(500.milliseconds),
//            InstantCommand { activateColorSensorDetection.cancel() },
            FollowPath(score3, holdEnd = true),
            Delay(2.seconds),
//            waitUntilReadyToShoot,
//            shootAllArtifacts(200.milliseconds),
            FollowPath(park),
        )

        autonomousRoutine()
    }

    override fun onUpdate() {
        telemetry.addData("x", PedroComponent.follower.pose.x)
        telemetry.addData("y", PedroComponent.follower.pose.y)
        telemetry.addData("heading", PedroComponent.follower.pose.heading)
        draw()
    }

    val horizontalIntakeStart = 100.0
    val horizontalIntakeEnd   = 117.0

    val intake1Vertical = 34.0
    val intake2Vertical = 58.5
    val intake3Vertical = 83.0

    val intakeHeading = Math.toRadians(0.0)

    val startPose     = Pose(88.0, 8.5, Math.toRadians(90.0))
    val scorePose     = Pose(88.0, 14.0, Math.toRadians(65.0))
    val parkPose      = Pose(105.500, 33.500, Math.toRadians(90.0))
    val startIntake1  = Pose(horizontalIntakeStart, intake1Vertical, intakeHeading)
    val endIntake1    = Pose(horizontalIntakeEnd,   intake1Vertical, intakeHeading)
    val startIntake2  = Pose(horizontalIntakeStart, intake2Vertical, intakeHeading)
    val endIntake2    = Pose(horizontalIntakeEnd,   intake2Vertical, intakeHeading)
    val startIntake3  = Pose(horizontalIntakeStart, intake3Vertical, intakeHeading)
    val endIntake3    = Pose(horizontalIntakeEnd,   intake3Vertical, intakeHeading)

    lateinit var scorePreload: PathChain
    lateinit var intake1: PathChain
    lateinit var score1: PathChain
    lateinit var intake2: PathChain
    lateinit var score2: PathChain
    lateinit var intake3: PathChain
    lateinit var score3: PathChain
    lateinit var park: PathChain

    fun buildPaths() {
        scorePreload = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    startPose,
                    scorePose
                )
            )
            .setLinearHeadingInterpolation(startPose.heading, scorePose.heading)
            .build()

        intake1 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    scorePose,
                    Pose(startPose.x, intake1Vertical),
                    startIntake1
                )
            )
            .setLinearHeadingInterpolation(scorePose.heading, startIntake1.heading)
            .addPath(
                BezierLine(startIntake1, endIntake1)
            )
            .setTangentHeadingInterpolation()
            .build()
        
        // spin spindexer

        score1 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(endIntake1, scorePose)
            )
            .setLinearHeadingInterpolation(endIntake1.heading, scorePose.heading)
            .build()
        
        // shoot + aim

        intake2 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    scorePose,
                    Pose(scorePose.x, intake2Vertical),
                    startIntake2
                )
            )
            .setLinearHeadingInterpolation(scorePose.heading, startIntake2.heading)
            .addPath(
                BezierLine(startIntake2, endIntake2)
            )
            .setTangentHeadingInterpolation()
            .build()

        score2 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(endIntake2, scorePose)
            )
            .setLinearHeadingInterpolation(endIntake2.heading, scorePose.heading)
            .build()

        intake3 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    scorePose,
                    Pose(scorePose.x, intake3Vertical),
                    startIntake3
                )
            )
            .setLinearHeadingInterpolation(scorePose.heading, startIntake3.heading)
            .addPath(
                BezierLine(startIntake3, endIntake3)
            )
            .setTangentHeadingInterpolation()
            .build()

        score3 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(endIntake3, scorePose)
            )
            .setLinearHeadingInterpolation(endIntake3.heading, scorePose.heading)
            .build()

        park = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(scorePose, parkPose)
            )
            .setLinearHeadingInterpolation(scorePose.heading, parkPose.heading)
            .build()
    }
}
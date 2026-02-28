package org.firstinspires.ftc.teamcode.opmodes.teleop

import android.util.Log
import com.bylazar.configurables.annotations.Configurable
import com.bylazar.field.Style
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.utils.LoopTimer
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.HeadingInterpolator
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
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
import org.firstinspires.ftc.teamcode.util.MotifPattern
import org.firstinspires.ftc.teamcode.util.TelemetryImplUpstreamSubmission
import org.firstinspires.ftc.teamcode.util.auto.AutoPoses
import kotlin.math.abs

@Configurable
@TeleOp(name = "Combined TeleOp")
class CombinedTeleOp : LinearOpMode() {
    companion object {
        var follower: Follower? = null
        var motifPattern: MotifPattern? = null
        var alliance: Alliance? = null

        @JvmField
        var lowpass = 0.2

        const val TUNING_FLYWHEEL = false
        const val DEBUG_FSM = true
    }
    enum class Shoot {
        IDLE,
        MOVE_SPINDEXER,
        TRANSFER_ARTIFACT,
        WAIT_FOR_COMPLETION
    }

    private var shootingState = Shoot.IDLE
    val timer = ElapsedTime()
    val matchTimer = ElapsedTime()

    var artifactDetected = false
    var lastArtifactDetected = false

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
                shootingState = Shoot.TRANSFER_ARTIFACT
                timer.reset()
            }

            Shoot.TRANSFER_ARTIFACT   -> {
                if (DEBUG_FSM) {
                    Log.d("FSM", "          Waiting for shooter: ${shooter.atSetPoint()}\n          Waiting for spindexer: ${spindexer.atSetPoint()}")
                    Log.d("FSM", "          spindexer: ${spindexer.currentAngle}, ${spindexer.targetAngle},\n" +
                            "          shooter: ${shooter.flywheelRPM}, ${shooter.targetFlywheelRPM}")
                }

                if (shooter.atSetPoint() && spindexer.atSetPoint()) {
                    if (DEBUG_FSM) Log.d("FSM", "Moving spindexer / shooter took ${timer.milliseconds()}")
                    transfer.transferArtifact()
                    if (DEBUG_FSM) {
                        Log.d("FSM", "= = = Transferring = = =")
                        Log.d("FSM", "RPM: ${shooter.flywheelRPM}, Hood angle: ${shooter.hoodPosToDegrees(shooter.hoodPosition)}")
                    }
                    shootingState = Shoot.WAIT_FOR_COMPLETION
                    timer.reset()
                }
            }

            Shoot.WAIT_FOR_COMPLETION -> {
                if (DEBUG_FSM) Log.d("FSM", "          Waiting for transfer, current: ${transfer.currentPosition}, target: ${transfer.targetPosition}, diff: ${transfer.targetPosition - transfer.currentPosition}")
                if (transfer.atSetPoint()) {
                    if (DEBUG_FSM) Log.d("FSM", "transferring took ${timer.milliseconds()}")
                    spindexer.recordOuttake()
                    if (DEBUG_FSM) {
                        Log.d("FSM", "EVALUATING SPINDEXER FULLNESS")
                        Log.d("FSM", "Spindexer isEmpty: " + spindexer.isEmpty + ", isFull: " + spindexer.isFull + ", Str: " + spindexer.getArtifactString())
                    }

                    timer.reset()
                    if (spindexer.isEmpty) {
                        shootingState = Shoot.IDLE
                        gamepad1.rumble(250)
                        spindexer.toFirstEmptyIntakePosition()
                    } else {
                        shootingState = Shoot.MOVE_SPINDEXER
                    }
                }
            }

            Shoot.IDLE                -> {}
        }
    }

    val intake = Intake()
    val transfer = Transfer()
    val spindexer = Spindexer()
    val shooter = Shooter()
    val turret = Turret()
    val camera = OV9281()
    val colorSensor = ColorSensor()
    val subsystems = listOf(
        intake,
        transfer,
        spindexer,
        shooter,
        turret,
        camera,
        colorSensor
    )

    var lastSpindexerIsFull = false

    private val spindexerFullRumbleEffect: Gamepad.RumbleEffect by lazy {
        Gamepad.RumbleEffect.Builder()
            .addStep(0.0, 1.0, 167)
            .addStep(1.0, 0.0, 167)
            .addStep(0.0, 1.0, 167)
            .build()
    }

    private var futurePose = Pose()

    override fun runOpMode() {
        val fol = follower ?: Constants.createFollower(hardwareMap).apply {
            setStartingPose(Pose(72.0,72.0,Math.PI/2))
            follower = this
        }

        fol.breakFollowing()
        fol.setMaxPower(1.0)

        spindexer.motifPattern = motifPattern
        gamepad1.triggerThreshold = 0.15f
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, TelemetryImplUpstreamSubmission(this))
        val loopTimer = LoopTimer(10)

        subsystems.forEach { it.initialize() }
        fol.update()
        Drawing.init()
        while (opModeInInit()) {
            telemetry.update()
        }

        val autoPoses = AutoPoses(alliance ?: Alliance.RED)
        val goToFarShoot = {
            fol.pathBuilder()
                .addPath(BezierLine(fol::getPose, Pose(88.0, 14.0)))
                .setHeadingInterpolation(
                    HeadingInterpolator.linearFromPoint(
                        fol::getHeading,
                        Math.toRadians(90.0),
                        1.0
                    )
                )
                .build()
        }

        val goToNearShoot = {
            fol.pathBuilder()
                .addPath(BezierLine(fol::getPose, Pose(88.0, 88.0)))
                .setHeadingInterpolation(
                    HeadingInterpolator.linearFromPoint(
                        fol::getHeading,
                        Math.toRadians(0.0),
                        1.0
                    )
                )
                .build()
        }

        val goToPark = {
            fol.pathBuilder()
                .addPath(
                    BezierLine(
                        fol::getPose,
                        if (alliance == Alliance.RED) autoPoses.redPark else autoPoses.redPark.mirror()
                    )
                )
                .setConstantHeadingInterpolation(autoPoses.redPark.heading)
                .build()
        }


        // bulk caching
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        allHubs.forEach { hub -> hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        turret.selectedAlliance = alliance ?: Alliance.RED
        loopTimer.start()

        var turretAutomate = true

        fol.startTeleopDrive(true)
        matchTimer.reset()

        while (opModeIsActive()) {
            loopTimer.end()
            loopTimer.start()
            // more bulk caching
            allHubs.forEach { hub -> hub.clearBulkCache() }

            // gets it until it is gotten :tm:
            if (motifPattern == null && camera.detectionsAmount > 0) {
                motifPattern = camera.motif
                spindexer.motifPattern = motifPattern
            }

            // transfer
            if (gamepad1.triangleWasPressed() && spindexer.atSetPoint()) {
                transfer.transferArtifact()
                spindexer.recordOuttake()
            }

            // spindexer
            if (gamepad1.leftBumperWasPressed() && transfer.atSetPoint()) {
                spindexer.toNextOuttakePosition()
            }

            if (gamepad1.rightBumperWasPressed() && transfer.atSetPoint()) {
                spindexer.toNextIntakePosition()
            }

            // intake
            if (gamepad1.squareWasPressed()) intake.toggle()

            // while held, reversed
            if (gamepad1.aWasPressed()) {
                intake.reversed = true
            }

            if (gamepad1.aWasReleased()) {
                intake.reversed = false
            }

            fol.setTeleOpDrive(
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble(),
                true
            )

            if (gamepad1.dpadUpWasPressed() && !turretAutomate) {
                shooter.setTargetState(autoPoses.farShootTeleopOwnGate.distanceFrom(turret.goalPose))
                turret.robotPose = autoPoses.farShootTeleopOwnGate
            }

            if (gamepad1.dpadDownWasPressed() && !turretAutomate) {
                shooter.setTargetState(autoPoses.farShootTeleopHP.distanceFrom(turret.goalPose))
                turret.robotPose = autoPoses.farShootTeleopHP
            }

            if (gamepad1.circleWasPressed()) {
                spindexer.toFirstEmptyIntakePosition()
                shootingState = Shoot.IDLE
            }

            // update all mechanisms

            if (gamepad1.rightTriggerWasPressed() && transfer.atSetPoint()) {
                shootAllArtifacts()
            }

            if (TUNING_FLYWHEEL) {
                if (gamepad1.dpadDownWasPressed()) {
                    shooter.targetFlywheelRPM -= 125
                }

                if (gamepad1.dpadUpWasPressed()) {
                    shooter.targetFlywheelRPM += 125
                }

                if (gamepad1.dpadRightWasPressed()) {
                    shooter.hoodPosition += 0.025
                }

                if (gamepad1.dpadLeftWasPressed()) {
                    shooter.hoodPosition -= 0.025
                }
            }

            if (gamepad1.touchpadWasPressed()) {
                turretAutomate = !turretAutomate
            }

            lastArtifactDetected = artifactDetected
            artifactDetected =
                colorSensor.detectedArtifact != null && !spindexer.isFull && spindexer.slotsToIntakes.contains(
                    spindexer.state
                ) && spindexer.atSetPoint() && intake.power != Intake.State.OFF
            if (artifactDetected && !lastArtifactDetected) {
                spindexer.recordIntake(colorSensor.detectedArtifact!!)
                spindexer.toFirstEmptyIntakePosition()
                colorSensor.detectedArtifact = null
            }

            if (camera.hasNewReading && gamepad1.left_trigger > 0.2) {
                fol.pose = Pose(
                    (1-lowpass) * fol.pose.x + lowpass * camera.robotPose.x,
                    (1-lowpass) * fol.pose.y + lowpass * camera.robotPose.y,
                    (1-lowpass) * fol.pose.heading + lowpass * camera.robotPose.heading
                )
            }

            if (gamepad1.dpadLeftWasPressed()) spindexer.decreaseOffset()
            if (gamepad1.dpadRightWasPressed()) spindexer.increaseOffset()
            if (spindexer.isFull && !lastSpindexerIsFull) {
                gamepad1.rumble(500)
            }

//            // timer rumble!!
//            if (timerForWhile(matchTimer, 100.0, 1.0)) {
//                gamepad1.rumble(100)
//            }
//
//            if (timerForWhile(matchTimer, 110.0, 1.0)) {
//                gamepad1.rumble(100)
//            }
//
//            if (timerForWhile(matchTimer, 115.0, 5.0)) {
//                gamepad1.rumble(100)
//            }

            updateShootingFSM()
            val lastFolVel = fol.velocity.magnitude
            val lastFolAcc = fol.acceleration.magnitude
            val lastFolPose = fol.pose
            fol.update()

//            if (!fol.pose.roughlyEquals(lastFolPose, lastFolVel * (loopTimer.ms/1000.0) + 0.5 *  (lastFolAcc * (loopTimer.ms/1000.0) * (loopTimer.ms/1000.0)) + 1.0)) {
//                fol.pose = lastFolPose
//            }
            // like repeat a bit yknow

            var lastInAirTime: Double
            do {
                futurePose = Pose(
                    fol.pose.x + shooter.expectedTimeInAir * fol.velocity.xComponent,
                    fol.pose.y + shooter.expectedTimeInAir * fol.velocity.yComponent,
                    fol.pose.heading
                )
                lastInAirTime = shooter.expectedTimeInAir

                if (!TUNING_FLYWHEEL && turretAutomate) {
                    shooter.setTargetState(turret.goalPose.distanceFrom(futurePose))
                } else {
                    shooter.setTimeInAir(turret.goalPose.distanceFrom(futurePose))
                }
            } while (abs(lastInAirTime - shooter.expectedTimeInAir) > 0.001)

            futurePose = Pose(
                fol.pose.x + shooter.expectedTimeInAir * fol.velocity.xComponent,
                fol.pose.y + shooter.expectedTimeInAir * fol.velocity.yComponent,
                fol.pose.heading
            )

            if (turretAutomate) {
                turret.robotPose = futurePose
            }

            Drawing.drawRobot(
                    fol.pose,
                    Style(
                        "",
                        "#FFFFFF",
                        0.75
                    )
                )
                Drawing.drawRobot(
                    futurePose,
                    Style(
                        "",
                        "#FF881E",
                        0.75
                    )
                )
                Drawing.sendPacket()

            lastSpindexerIsFull = spindexer.isFull
            subsystems.forEach { it.periodic() }
            telemetry.run{
                addData("Loop ms", "%05.2f", loopTimer.ms.toDouble())
                addData("x", fol.pose.x)
                addData("y", fol.pose.y)
                addData("heading", fol.pose.heading)
                addData("Time Elapsed", matchTimer.seconds())
                update()
            }
        }
    }

    fun timerForWhile(timer: ElapsedTime, seconds: Double, trueTime: Double): Boolean {
        return timer.seconds() in seconds..(seconds+trueTime)
    }
}
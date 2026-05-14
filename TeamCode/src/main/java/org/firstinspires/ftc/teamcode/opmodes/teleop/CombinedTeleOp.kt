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
import com.skeletonarmy.marrow.prompts.BooleanPrompt
import com.skeletonarmy.marrow.prompts.Prompter
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
import org.firstinspires.ftc.teamcode.util.normalizeRadians
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

        const val DEBUG_FSM = true
    }
    var TUNING_FLYWHEEL = false

    var turretAutomate = true

    var artifactDetected = false
    var lastArtifactDetected = false

    private var shootingState = Shoot.IDLE
    val timer = ElapsedTime()
    val matchTimer = ElapsedTime()
    val lastShotTimer = ElapsedTime()
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
                intake.intake()
                if (DEBUG_FSM) Log.d("FSM", " * * * * * NEW CYCLE: * * * * * moving spindexer to ${spindexer.state.name}, ${spindexer.getArtifactString()}")
                timer.reset()
                shootingState = Shoot.WAIT_FOR_COMPLETION
            }

            Shoot.WAIT_FOR_COMPLETION -> {
                if (DEBUG_FSM) Log.d("FSM", "          Waiting for spindexer, current: ${spindexer.currentAngle}, target: ${spindexer.targetAngle}, diff: ${spindexer.targetAngle - spindexer.currentAngle}")
                if (spindexer.atSetPoint()) {
                    if (DEBUG_FSM) Log.d("FSM", "spindexer took ${timer.milliseconds()} to rotate")
                    transfer.on() // assume instantaneous transfer
                    spindexer.recordOuttake()

                    if (DEBUG_FSM) {
                        Log.d("FSM", "EVALUATING SPINDEXER FULLNESS")
                        Log.d("FSM", "Spindexer isEmpty: " + spindexer.isEmpty + ", isFull: " + spindexer.isFull + ", Str: " + spindexer.getArtifactString())
                    }

                    timer.reset()
                    lastShotTimer.reset()
                    shootingState = Shoot.WAIT_FOR_LAST_SHOT
                }
            }

            Shoot.WAIT_FOR_LAST_SHOT -> {
                if (DEBUG_FSM) Log.d("FSM", "waiting for last shot to clear. ${ 50 - lastShotTimer.milliseconds()} remaining")
                if (shooter.atSetPoint() || (lastShotTimer.milliseconds() >= 2500.0) ){
                    shootingState = if (spindexer.isEmpty) {
                        lastlastshottimer.reset()
                        Shoot.LAST_SHOT_DELAY
                    } else {
                        Shoot.MOVE_SPINDEXER
                    }
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

    val intake = Intake()
    val transfer = Transfer()
    val spindexer = Spindexer()
    val shooter = Shooter()
    val turret = Turret()
//    val camera = OV9281()
    val colorSensor = ColorSensor()
    val subsystems = listOf(
        intake,
        transfer,
        spindexer,
        turret,
//        camera,
        colorSensor,
        shooter,
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
        val prompter = Prompter(this)

        prompter
            .prompt("tf", BooleanPrompt("TUNING FLYWHEEL", false))
            .onComplete {
                TUNING_FLYWHEEL = prompter.get<Boolean>("tf")
                telemetry.addLine("six seven")
            }
        Drawing.init()
        while (opModeInInit()) {
            telemetry.update()
            prompter.run()
            fol.update()
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

        fol.startTeleopDrive(true)
        matchTimer.reset()

        if (!turretAutomate) turret.automatic = false; turret.angle = 0.0
        while (opModeIsActive()) {
            loopTimer.end()
            loopTimer.start()
            // more bulk caching
            allHubs.forEach { hub -> hub.clearBulkCache() }

            if (gamepad1.backWasPressed()) turret.automatic = !turret.automatic

            if (gamepad1.dpadDownWasPressed()) {
                motifPattern = MotifPattern.PGP
                spindexer.motifPattern = motifPattern
            }

            if (gamepad1.dpadLeftWasPressed()) {
                motifPattern = MotifPattern.GPP
                spindexer.motifPattern = motifPattern
            }

            if (gamepad1.dpadRightWasPressed()) {
                motifPattern = MotifPattern.PPG
                spindexer.motifPattern = motifPattern
            }

            // gets it until it is gotten :tm:
//            if (motifPattern == null && camera.detectionsAmount > 0) {
//                motifPattern = camera.motif
//                spindexer.motifPattern = motifPattern
//            }

            if (shootingState == Shoot.IDLE) {
                transfer.transferOn = gamepad1.triangle
            }

            if (spindexer.slotsToOuttakes.contains(spindexer.state) && spindexer.atSetPoint() && transfer.transferOn && shootingState == Shoot.IDLE) {
                spindexer.recordOuttake()
            }

            // spindexer
            if (gamepad1.leftBumperWasPressed()) {
                spindexer.toNextOuttakePosition()
            }

            if (gamepad1.rightBumperWasPressed()) {
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

//            if (gamepad1.dpadUpWasPressed() && !turretAutomate) {
//                shooter.setTargetState(autoPoses.farShootTeleopOwnGate.distanceFrom(turret.goalPose))
//                turret.robotPose = autoPoses.farShootTeleopOwnGate
//            }
//
//            if (gamepad1.dpadDownWasPressed() && !turretAutomate) {
//                shooter.setTargetState(autoPoses.farShootTeleopHP.distanceFrom(turret.goalPose))
//                turret.robotPose = autoPoses.farShootTeleopHP
//            }

            if (gamepad1.circleWasPressed()) {
                endShootingCycle()
            }

            // update all mechanisms

            if (gamepad1.rightTriggerWasPressed()) {
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

            if (gamepad1.leftTriggerWasPressed()) {
                fol.pose = if (alliance == Alliance.RED ) Pose(126.0,83.5,Math.toRadians(90.0)) else Pose(18.0,83.5, Math.toRadians(90.0))
            }

//            if (camera.hasNewReading && gamepad1.left_trigger > 0.2 && camera.bufferSize == 3) {
//                val correctedRobotPose = turret.turretPoseToRobotPose(camera.turretPose)
//
//                 Calculate the shortest path difference between the two angles
//                val headingDelta = normalizeRadians(
//                    correctedRobotPose.heading - fol.pose.heading,
//                    false
//                )
//
//                 Apply the lowpass to the delta, then add it to the current heading
//                val newHeading = normalizeRadians(
//                    fol.pose.heading + (lowpass * headingDelta),
//                    true
//                )
//
//                fol.pose = Pose(
//                    (1-lowpass) * fol.pose.x + lowpass * correctedRobotPose.x,
//                    (1-lowpass) * fol.pose.y + lowpass * correctedRobotPose.y,
//                    if (gamepad1.right_stick_button) newHeading else fol.pose.heading
//                )
//            }
            
            if (spindexer.isFull && !lastSpindexerIsFull) {
                gamepad1.rumble(500)
                spindexer.toMotifOuttakePosition()
                intake.off()
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
            fol.update()

            // like repeat a bit yknow

            var lastInAirTime: Double
            do {
                futurePose = Pose(
                    fol.pose.x + shooter.expectedTimeInAir * fol.velocity.xComponent,
                    fol.pose.y + shooter.expectedTimeInAir * fol.velocity.yComponent,
                    fol.pose.heading
                )
                lastInAirTime = shooter.expectedTimeInAir

                if (!TUNING_FLYWHEEL) {
                    shooter.setTargetState(turret.goalPose.distanceFrom(futurePose))
                } else {
                    shooter.setTimeInAir(turret.goalPose.distanceFrom(futurePose))
                }
            } while (abs(lastInAirTime - shooter.expectedTimeInAir) > 0.0001)

            futurePose = Pose(
                fol.pose.x + shooter.expectedTimeInAir * fol.velocity.xComponent,
                fol.pose.y + shooter.expectedTimeInAir * fol.velocity.yComponent,
                fol.pose.heading
            )

            if (turretAutomate) {
                turret.robotPose = futurePose
            }

//            val correctedRobotPose = turret.turretPoseToRobotPose(camera.turretPose)
//            Drawing.drawRobot(
//                camera.turretPose,
//                Style(
//                    "",
//                    "#0000FF",
//                    0.75
//                )
//            )
//            Drawing.drawRobot(
//                    fol.pose,
//                    Style(
//                        "",
//                        "#FFFFFF",
//                        0.75
//                    )
//                )
//            Drawing.drawRobot(
//                correctedRobotPose,
//                Style(
//                    "",
//                    "#FF881E",
//                    0.75
//                )
//            )
//            Drawing.drawRobot(
//                turret.goalPose,
//                Style(
//                    "",
//                    "#FFFF00",
//                    0.5
//                )
//            )
//            Drawing.sendPacket()
//
            lastSpindexerIsFull = spindexer.isFull
            subsystems.forEach { it.periodic() }

            telemetry.run{
                addData("Loop ms", "%05.2f", loopTimer.ms.toDouble())
                addData("x", fol.pose.x)
                addData("y", fol.pose.y)
                addData("heading", fol.pose.heading)
                addData("Time Elapsed", matchTimer.seconds())
//                addData("Camera pose adj", "x: %05.2f, y: %05.2f, h: %05.2f", correctedRobotPose.x, correctedRobotPose.y,
//                    correctedRobotPose.heading)
//                addData("Camera pose", "x: %05.2f, y: %05.2f, h: %05.2f", camera.turretPose.x, camera.turretPose.y,
//                    camera.turretPose.heading)
                update()
            }
        }
    }

    fun timerForWhile(timer: ElapsedTime, seconds: Double, trueTime: Double): Boolean {
        return timer.seconds() in seconds..(seconds+trueTime)
    }
}
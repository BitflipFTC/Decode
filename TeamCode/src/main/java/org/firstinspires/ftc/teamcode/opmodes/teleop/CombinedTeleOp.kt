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
import org.firstinspires.ftc.teamcode.util.InitConfigurer
import org.firstinspires.ftc.teamcode.util.MotifPattern
import org.firstinspires.ftc.teamcode.util.auto.AutoPoses

@Configurable
@TeleOp(name = "Combined TeleOp")
class CombinedTeleOp : LinearOpMode() {
    companion object {
        var follower: Follower? = null
        var motifPattern: MotifPattern? = null

        @JvmField
        var lowpass = 0.1
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
                Log.d("FSM", "MOVING SPINDEXER TO ${spindexer.state.name}, ${spindexer.getArtifactString()}")
                shootingState = Shoot.TRANSFER_ARTIFACT
                timer.reset()
            }

            Shoot.TRANSFER_ARTIFACT   -> {
                Log.d("FSM", "Waiting for shooter or spindexer")
                Log.d("FSM", "sp: ${spindexer.currentAngle}, ${spindexer.targetAngle}, sh: ${shooter.flywheelRPM}, ${shooter.targetFlywheelRPM}")
                if (shooter.atSetPoint() && spindexer.atSetPoint()) {
                    Log.d("FSM", "Moving spindexer / shooter took ${timer.milliseconds()}")
                    transfer.transferArtifact()
                    Log.d("FSM", "TRANSFERING")
                    shootingState = Shoot.WAIT_FOR_COMPLETION
                    timer.reset()
                }
            }

            Shoot.WAIT_FOR_COMPLETION -> {
                Log.d("FSM", "WAITING FOR TRANSFER, current: ${transfer.currentPosition}, target: ${transfer.targetPosition}, diff: ${transfer.targetPosition - transfer.currentPosition}")
                if (transfer.atSetPoint()) {
                    Log.d("FSM", "transferring took ${timer.milliseconds()}")
                    spindexer.recordOuttake()
                    Log.d("FSM", "EVALUATING SPINDEXER FULLNESS")
                    Log.d("FSM", "Spindexer isEmpty: " + spindexer.isEmpty + ", isFull: " + spindexer.isFull + ", Str: " + spindexer.getArtifactString())

                    timer.reset()
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

    var reverseIntake = false
    var lastReverseIntake = false
    var lastSpindexerIsFull = false
    var tuningFlywheel = false

    override fun runOpMode() {
        val fol = follower ?: Constants.createFollower(hardwareMap).apply {
            setStartingPose(Pose(72.0,72.0,Math.toRadians(90.0)))
            follower = this
        }

        spindexer.motifPattern = motifPattern
        gamepad1.triggerThreshold = 0.15f
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
        val loopTimer = LoopTimer(10)

        subsystems.forEach { it.initialize() }
        fol.update()
        Drawing.init()
        var automatedDriving = false

        InitConfigurer.preInit()
        while (opModeInInit()) {
            InitConfigurer.postWaitForStart()
            telemetry.update()
        }

        val autoPoses = AutoPoses(InitConfigurer.selectedAlliance ?: Alliance.RED)
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
                        if (InitConfigurer.selectedAlliance == Alliance.RED) autoPoses.redPark else autoPoses.redPark.mirror()
                    )
                )
                .setConstantHeadingInterpolation(autoPoses.redPark.heading)
                .build()
        }


        // bulk caching
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        allHubs.forEach { hub -> hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        turret.selectedAlliance = InitConfigurer.selectedAlliance ?: Alliance.RED
        loopTimer.start()

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
            if (gamepad1.leftBumperWasPressed() && transfer.atSetPoint() && intake.power != Intake.State.OFF) {
                spindexer.toNextOuttakePosition()
            }

            if (gamepad1.rightBumperWasPressed() && transfer.atSetPoint() && intake.power != Intake.State.OFF) {
                spindexer.toNextIntakePosition()
            }

            // intake
            if (gamepad1.squareWasPressed()) intake.toggle()

            // while held, reversed
            if (gamepad1.leftTriggerWasPressed()) {
                intake.reversed = true
            }
//
            if (gamepad1.leftTriggerWasReleased()) {
                intake.reversed = false
            }

            if (!automatedDriving) {
                fol.setTeleOpDrive(
                    -gamepad1.left_stick_y.toDouble(),
                    -gamepad1.left_stick_x.toDouble(),
                    -gamepad1.right_stick_x.toDouble(),
                    true
                )
            }

            if (gamepad1.backWasPressed()) {
                automatedDriving = true
                fol.followPath(goToPark(), true)
                gamepad1.setLedColor(0.0,0.0,255.0, Gamepad.LED_DURATION_CONTINUOUS)
            }

//            if (gamepad1.optionsWasPressed()) {
//                fol.pose = if (turret.selectedAlliance == Alliance.RED) nearStartPose else nearStartPose.mirror()
//            }

            if (automatedDriving && (!fol.isBusy || gamepad1.circleWasPressed())) {
                automatedDriving = false
                fol.startTeleopDrive(true)
                gamepad1.setLedColor(255.0, 136.0, 30.0, Gamepad.LED_DURATION_CONTINUOUS)
            }

            if (gamepad1.circleWasPressed()) {
                spindexer.toFirstEmptyIntakePosition()
                shootingState = Shoot.IDLE
            }

            // update all mechanisms

            if (gamepad1.rightTriggerWasPressed() && transfer.atSetPoint()) {
                shootAllArtifacts()
                gamepad1.rumble(250)
            }

            if (!tuningFlywheel) {
                if (gamepad1.dpadRightWasPressed()) {
                    spindexer.increaseOffset()
                }

                if (gamepad1.dpadLeftWasPressed()) {
                    spindexer.decreaseOffset()
                }
            }

            if (tuningFlywheel) {
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
//
//            if (gamepad1.dpadLeftWasPressed()) {
//                spindexer.motifPattern = MotifPattern.GPP
//                motifPattern = MotifPattern.GPP
//            }
//
//            if (gamepad1.dpadUpWasPressed()) {
//                spindexer.motifPattern = MotifPattern.PGP
//                motifPattern = MotifPattern.PGP
//            }
//
//            if (gamepad1.dpadRightWasPressed()) {
//                spindexer.motifPattern = MotifPattern.PPG
//                motifPattern = MotifPattern.PPG
//            }
//
            if (!tuningFlywheel) {
                shooter.setTargetState(turret.goalPose.distanceFrom(fol.pose))
            }

            lastArtifactDetected = artifactDetected
            artifactDetected =
                colorSensor.detectedArtifact != null && !spindexer.isFull && spindexer.slotsToIntakes.contains(
                    spindexer.state
                ) && spindexer.atSetPoint() && intake.power != Intake.State.OFF
            if (artifactDetected && !lastArtifactDetected) {
//                Log.d("FSM", "detected artifact: ${localDetectedArtifact?.name}, spindexer not full: ${!spindexer.isFull}, spindexer state not null: ${spindexer.slotsToIntakes.contains(spindexer.state)}, spindexer at set point: ${spindexer.atSetPoint()}")
                spindexer.recordIntake(colorSensor.detectedArtifact!!)
//                Log.d("FSM","new artifact string: ${spindexer.getArtifactString()}, current state: ${spindexer.state.name}")
                spindexer.toFirstEmptyIntakePosition()
                gamepad1.rumbleBlips(spindexer.totalFullSlots)
                colorSensor.detectedArtifact = null
            }

            lastSpindexerIsFull = spindexer.isFull

            if (camera.hasNewReading && fol.velocity.magnitude < 1.0 && fol.angularVelocity < 1 * ((2 * Math.PI) / 360) ) {
                fol.pose = Pose(
                    (1-lowpass) * fol.pose.x + lowpass * camera.robotPose.x,
                    (1-lowpass) * fol.pose.y + lowpass * camera.robotPose.y,
                    (1-lowpass) * fol.pose.heading + lowpass * camera.robotPose.heading
                )
            }

            // timer rumble!!






            updateShootingFSM()
            fol.update()
            Drawing.drawDebug(fol)
            turret.robotPose = fol.pose
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
}
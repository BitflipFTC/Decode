package org.firstinspires.ftc.teamcode.opmodes.teleop

import android.util.Log
import com.bylazar.configurables.annotations.Configurable
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
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.opmodes.auto.Red12
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing
import org.firstinspires.ftc.teamcode.subsystems.ArtifactColorSensor
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Spindexer
import org.firstinspires.ftc.teamcode.subsystems.Transfer
import org.firstinspires.ftc.teamcode.subsystems.Turret
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.OV9281
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.InitConfigurer
import org.firstinspires.ftc.teamcode.util.MotifPattern
import org.firstinspires.ftc.teamcode.util.dpadToAxes
import kotlin.math.abs

@Configurable
@TeleOp(name = "Combined TeleOp")
class CombinedTeleOp : LinearOpMode() {
    companion object {
        var follower: Follower? = null
        var motifPattern: MotifPattern? = null
    }

    enum class Shoot {
        IDLE,
        MOVE_SPINDEXER,
        TRANSFER_ARTIFACT,
        WAIT_FOR_COMPLETION
    }

    private var shootingState = Shoot.IDLE

    var artifactDetected = false
    var lastArtifactDetected = false

    var lastSpindexerFull = false

    fun shootAllArtifacts() {
        if (!spindexer.isEmpty) {
            shootingState = Shoot.MOVE_SPINDEXER
        }
    }

    fun updateShootingFSM () {
        when (shootingState) {
            Shoot.MOVE_SPINDEXER    -> {
                spindexer.toMotifOuttakePosition()
//                Log.d("FSM", "MOVING SPINDEXER TO ${spindexer.state.name}, ${spindexer.getArtifactString()}")
                shootingState = Shoot.TRANSFER_ARTIFACT
            }
            Shoot.TRANSFER_ARTIFACT -> {
//                Log.d("FSM", "Waiting for shooter or spindexer")
//                Log.d("FSM", "sp: ${spindexer.currentAngle}, ${spindexer.targetAngle}, sh: ${shooter.flywheelRPM}, ${shooter.targetFlywheelRPM}")
                if (shooter.atSetPoint() && spindexer.atSetPoint()) {
                    transfer.transferArtifact()
//                    Log.d("FSM", "TRANSFERING")
                    shootingState = Shoot.WAIT_FOR_COMPLETION
                }
            }
            Shoot.WAIT_FOR_COMPLETION -> {
//                Log.d("FSM", "WAITING FOR TRANSFER, ${transfer.currentPosition}, ${transfer.targetPosition}")
                if (transfer.atSetPoint()) {
                    spindexer.recordOuttake()
//                    Log.d("FSM", "EVALUATING SPINDEXER FULLNESS")
//                    Log.d("FSM", "Spindexer isEmpty: " + spindexer.isEmpty + ", isFull: " + spindexer.isFull + ", Str: " + spindexer.getArtifactString())

                    if (spindexer.isEmpty) {
                        shootingState = Shoot.IDLE
                        spindexer.toFirstEmptyIntakePosition()
                    } else {
                        shootingState = Shoot.MOVE_SPINDEXER
                    }
                }
            }
            Shoot.IDLE -> {}
        }
    }

    val intake = Intake()
    val transfer = Transfer()
    val spindexer = Spindexer()
    val shooter = Shooter()
    val turret = Turret()
       val camera = OV9281()
    val colorSensor = ArtifactColorSensor()
    val subsystems = listOf(
        intake,
        transfer,
        spindexer,
        shooter,
        turret,
        colorSensor,
        camera
    )

    override fun runOpMode() {
        if (follower == null) {
            follower = Constants.createFollower(hardwareMap)
            follower!!.setStartingPose(Pose(72.0, 72.0, Math.toRadians(90.0)))
        }
        spindexer.motifPattern = motifPattern
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
        val loopTimer = LoopTimer(10)

        subsystems.forEach { it.initialize() }
        follower!!.update()
        Drawing.init()
        var automatedDriving = false
        val goToFarShoot = {follower!!.pathBuilder()
            .addPath(BezierLine(follower!!::getPose, Pose(88.0, 14.0)))
            .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower!!::getHeading, Math.toRadians(90.0), 1.0))
            .build()}

        val goToNearShoot = {follower!!.pathBuilder()
            .addPath(BezierLine(follower!!::getPose, Pose(88.0, 88.0)))
            .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower!!::getHeading, Math.toRadians(0.0), 1.0))
            .build()}

        InitConfigurer.preInit()
        while (opModeInInit()) {
            InitConfigurer.postWaitForStart()
            telemetry.update()
        }

        // bulk caching
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        allHubs.forEach { hub -> hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        turret.selectedAlliance = InitConfigurer.selectedAlliance ?: Alliance.RED
//        camera.targetID = InitConfigurer.selectedAlliance?.aprilTagID ?: 24
        loopTimer.start()

//        follower.followPath(goToFarShoot())

        follower!!.startTeleopDrive(true)
        while (opModeIsActive()) {
            loopTimer.end()
            loopTimer.start()
            // more bulk caching
            allHubs.forEach { hub -> hub.clearBulkCache() }

            // see https://www.padcrafter.com/?templates=Driver+1%7CDriver+2&plat=1%7C1&col=%23242424%2C%23606A6E%2C%23FFFFFF&leftStick=Drivetrain+translation%7CTurret+Manual+Control&rightStick=Drivetrain+rotation%7CSpindexer+Manual+Control&yButton=Actuate+Transfer&xButton=Toggle+intake&rightTrigger=Auto+adjust+shooter%7CTransfer+forward&rightBumper=Spindexer%3A+Next+Outtake&leftBumper=Spindexer%3A+Next+Intake%7CAutoaim+turret&leftTrigger=Reverse+Intake%7CTransfer+backward&bButton=Reset+yaw+%28TO+BE+REMOVED%29%7CLong+shooting+preset&backButton=Toggle+field+centric&dpadLeft=%7CHoodPosition+-+0.05&dpadRight=%7CHoodPosition+%2B+0.05&dpadUp=%7CFlywheel+RPM+%2B+250&dpadDown=%7CFlywheel+RPM+-+250&aButton=%7CClose+shooting+preset
            // for gamepad layouts

            // transfer
            if (gamepad1.triangleWasPressed()) { transfer.transferArtifact(); gamepad1.rumble(500) }

            // spindexer
            if (gamepad1.leftBumperWasPressed()) {
                spindexer.toNextOuttakePosition()
                gamepad1.rumble(100)
            }

            if (gamepad1.rightBumperWasPressed()) {
                spindexer.toNextIntakePosition()
                gamepad1.rumble(100)
            }

            // intake
            if (gamepad1.squareWasPressed()) intake.toggle()

            // while held, reversed
            if (gamepad1.crossWasPressed()) {
                intake.reversed = true
            }
            if (gamepad1.crossWasReleased()) {
                intake.reversed = false
            }

            if (!automatedDriving) {
                follower!!.setTeleOpDrive(
                    -gamepad1.left_stick_y.toDouble(),
                    -gamepad1.left_stick_x.toDouble(),
                    -gamepad1.right_stick_x.toDouble(),
                    true
                )
            }

            if (gamepad1.backWasPressed()) {
                automatedDriving = true
                follower!!.followPath(goToFarShoot())
            }

            if (gamepad1.optionsWasPressed()) {
                automatedDriving = true
                follower!!.followPath(goToNearShoot())
            }

            if (automatedDriving && (!follower!!.isBusy || gamepad1.psWasPressed())) {
                automatedDriving = false
                follower!!.startTeleopDrive(true)
            }

            // update all mechanisms
            lastSpindexerFull = spindexer.isFull

            subsystems.forEach { it.periodic() }

            if (spindexer.isFull && !lastSpindexerFull) {
                intake.reversed = true
            } else if (!spindexer.isFull && lastSpindexerFull) {
                intake.reversed = false
            }
1
            if (gamepad1.touchpadWasPressed()) {
                shootAllArtifacts()
            }

//            if (gamepad1.dpadDownWasPressed()) {
//                shooter.targetFlywheelRPM -= 125
//            }
//
//            if (gamepad1.dpadUpWasPressed()) {
//                shooter.targetFlywheelRPM += 125
//            }
//
//            if (gamepad1.dpadRightWasPressed()) {
//                shooter.hoodPosition += 0.05
//            }
//
//            if (gamepad1.dpadLeftWasPressed()) {
//                shooter.hoodPosition -= 0.05
//            }

            shooter.setTargetState(turret.goalPose.distanceFrom(follower!!.pose))

//            turret.angle -= (gamepad1.right_trigger - gamepad1.left_trigger) * 5

            updateShootingFSM()

            lastArtifactDetected = artifactDetected
            artifactDetected = colorSensor.detectedArtifact != null && !spindexer.isFull && spindexer.slotsToIntakes.contains(spindexer.state) && spindexer.atSetPoint()
            if (artifactDetected && !lastArtifactDetected) {
//                Log.d("FSM", "detected artifact: ${colorSensor.detectedArtifact?.name}, spindexer not full: ${!spindexer.isFull}, spindexer state not null: ${spindexer.slotsToIntakes.contains(spindexer.state)}, spindexer at set point: ${spindexer.atSetPoint()}")
                spindexer.recordIntake(colorSensor.detectedArtifact!!)
//                Log.d("FSM","new artifact string: ${spindexer.getArtifactString()}, current state: ${spindexer.state.name}")
                spindexer.toFirstEmptyIntakePosition()
            }


            telemetry.addData("Loop Hz", "%05.2f", loopTimer.hz)
            telemetry.addData("Loop ms", "%05.2f", loopTimer.ms.toDouble())
            telemetry.addData("x", follower!!.pose.x)
            telemetry.addData("y", follower!!.pose.y)
            telemetry.addData("heading", follower!!.pose.heading)
            telemetry.addData("cam pose", camera.robotPose.toString())
            telemetry.update()

            follower!!.update()
            Drawing.drawDebug(follower)
            turret.robotPose = follower!!.pose
        }
    }
}
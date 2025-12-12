package org.firstinspires.ftc.teamcode.test

import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Const
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
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
import kotlin.math.PI
import kotlin.math.sin
import kotlin.math.sqrt

@TeleOp(name = "Shoot While Moving")
class ShootWhileMoving : LinearOpMode() {
    companion object {
        const val FLYWHEEL_DIAMETER = 0.072 // m
        const val GRAVITY = 9.80665 // m/s^2
        const val LINEAR_SPEED_COEFF = 0.45
        const val INITIAL_HEIGHT = 12.5 * 0.0254 // m
        const val TARGET_HEIGHT = 38.75 * 0.0254 // m
    }

    enum class Shoot {
        IDLE,
        MOVE_SPINDEXER,
        TRANSFER_ARTIFACT,
        CHECK_FOR_SHOT,
    }

    private var shootingState = Shoot.IDLE
    private var shootingRetryCounter = 0
    private val shootingRetryMax = 3

    fun shootAllArtifacts(spindexer: Spindexer) {
        if (!spindexer.isEmpty) {
            shootingState = Shoot.MOVE_SPINDEXER
        }
    }

    fun updateShootingFSM (spindexer: Spindexer, transfer: Transfer, shooter: Shooter) {
        when (shootingState) {
            Shoot.MOVE_SPINDEXER    -> {
                spindexer.toMotifOuttakePosition()
                shootingState = Shoot.TRANSFER_ARTIFACT
            }
            Shoot.TRANSFER_ARTIFACT -> {
                if (shooter.atSetPoint() && spindexer.atSetPoint()) {
                    transfer.transferArtifact()
                    shootingState = Shoot.CHECK_FOR_SHOT
                }
            }
            Shoot.CHECK_FOR_SHOT    -> {
                if (transfer.atSetPoint()) {
                    if (!shooter.atSetPoint()) { // outtake successful
                        spindexer.recordOuttake()
                    } else if (shootingRetryCounter < shootingRetryMax) { // outtake unsucessful, retry
                        shootingState = Shoot.TRANSFER_ARTIFACT
                        shootingRetryCounter++
                        return // leave this code block
                    }

                    // either outtake was successful or retries were exhausted
                    shootingRetryCounter = 0
                    shootingState = if (spindexer.isEmpty) Shoot.IDLE else Shoot.MOVE_SPINDEXER
                }
            }
            else              -> {}
        }
    }

    override fun runOpMode() {
        val intake = Intake()
        val camera = OV9281()
        val shooter = Shooter()
        val spindexer = Spindexer()
        val transfer = Transfer()
        val turret = Turret()
        val colorSensor = ArtifactColorSensor()
        val follower = Constants.createFollower(hardwareMap)

        val subsystems = setOf(intake, camera, shooter, spindexer, transfer, turret, colorSensor)

        var holdingPose = false

        subsystems.forEach { it.initialize() }

        camera.targetID = 24
        turret.selectedAlliance = Alliance.RED
        follower.setStartingPose(Pose(72.0, 72.0, Math.PI / 2))

        while (opModeInInit()) {
            follower.update()
        }

        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        allHubs.forEach { hub -> hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        BetterLoopTimeComponent.preStartButtonPressed()
        follower.startTeleopDrive(true)

        while (opModeIsActive()) {
            allHubs.forEach { hub -> hub.clearBulkCache() }

            if (!holdingPose) {
                follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * 0.8,
                    -gamepad1.left_stick_x * 0.8,
                    -gamepad1.right_stick_x * 0.8,
                    false
                )
            }

            if (gamepad1.squareWasPressed()) {
                intake.toggle()
            }

            if (gamepad1.crossWasPressed()) {
                intake.reversed = true
            }

            if (gamepad1.crossWasReleased()) {
                intake.reversed = false
            }

            if (gamepad1.triangleWasPressed()) {
                transfer.transferArtifact()
            }

            if (gamepad1.rightBumperWasPressed()) {
                spindexer.toFirstFullOuttakePosition()
            }

            if (gamepad1.leftBumperWasPressed()) {
                spindexer.toFirstEmptyIntakePosition()
            }

            if (colorSensor.detectedArtifact != null && !spindexer.isFull) {
                spindexer.recordIntake(colorSensor.detectedArtifact!!)
                spindexer.toFirstEmptyIntakePosition()
            }

            if (spindexer.isFull) {
                spindexer.toMotifOuttakePosition()
            }

            if (gamepad1.dpadUpWasPressed()) {
                shootAllArtifacts(spindexer)
            }

            if (gamepad1.dpadLeftWasPressed()) {
                spindexer.recordOuttake(0)
                spindexer.recordOuttake(1)
                spindexer.recordOuttake(2)
            }

            if (gamepad1.circleWasPressed()) {
                holdingPose = true
                follower.holdPoint(follower.pose)
            }

            if (gamepad1.circleWasReleased()) {
                holdingPose = false
                follower.breakFollowing()
            }

            // calc (short for calculations)
            val outtakeSpeed = Math.PI * FLYWHEEL_DIAMETER * (shooter.flywheelRPM / 60) * LINEAR_SPEED_COEFF // m/s
            val outtakeAngle = 90 - ((-46 * shooter.hoodPosition) + 45) // deg
            val verticalInitialSpeed = outtakeSpeed * sin(Math.toRadians(outtakeAngle)) // m/s

            // \frac{-\left(u_{y}\right)-\sqrt{\left(u_{y}\right)^{2}-4\left(-0.5g\right)\left(-h_{final}+h_{initial}\right)}}{2\left(-0.5g\right)}

            val a1 = (
                    -(verticalInitialSpeed) + sqrt((verticalInitialSpeed * verticalInitialSpeed) - 4 * (-0.5 * GRAVITY) * (-TARGET_HEIGHT + INITIAL_HEIGHT)) /
                    // --------------------------------------------------------------------------------------
                    2 * (-0.5 * GRAVITY)
                    )
            val a2 = (
                    -(verticalInitialSpeed) - sqrt((verticalInitialSpeed * verticalInitialSpeed) - 4 * (-0.5 * GRAVITY) * (-TARGET_HEIGHT + INITIAL_HEIGHT)) /
                            // --------------------------------------------------------------------------------------
                            2 * (-0.5 * GRAVITY)
                    )
            val shotTime = maxOf(a1, a2) // s
            telemetry.addData("Estimated shot time", shotTime)
            telemetry.addData("Apriltag robot pose", "%05.2fx, %05.2fy, %05.2fdeg", camera.robotPose.x, camera.robotPose.y, Math.toDegrees(camera.robotPose.heading))
            telemetry.addData("Odometry robot pose", "%05.2fx, %05.2fy, %05.2fdeg", follower.pose.x, follower.pose.y, Math.toDegrees(follower.pose.heading))

            val shotPose = Pose(follower.pose.x + follower.velocity.xComponent * shotTime, follower.pose.y + follower.velocity.yComponent * shotTime, follower.pose.heading)
            turret.robotPose = shotPose

            val shotDistance = turret.goalPose.distanceFrom(shotPose)
            shooter.setTargetState(shotDistance)
            // calc

            updateShootingFSM(spindexer,transfer,shooter)
            subsystems.forEach { it.periodic() }
            follower.update()
            BetterLoopTimeComponent.postUpdate()
            telemetry.update()
        }
    }
}
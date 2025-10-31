package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.hardware.Drivetrain
import org.firstinspires.ftc.teamcode.hardware.OV9281
import org.firstinspires.ftc.teamcode.hardware.Shooter
import org.firstinspires.ftc.teamcode.hardware.Spindexer
import org.firstinspires.ftc.teamcode.hardware.Transfer
import org.firstinspires.ftc.teamcode.hardware.Turret
import org.firstinspires.ftc.teamcode.util.Artifact
import org.firstinspires.ftc.teamcode.util.MotifPattern

@Config
@Autonomous(name = "Shoot Preloads Auto", group = "Autonomous", preselectTeleOp = "Combined TeleOp")
class ShootPreloadsAuto: LinearOpMode() {
    companion object {
        @JvmField var backupTime: Int = 2000
        @JvmField var rotateTime: Int = 750
        @JvmField var offLineTime = 1500
    }

    enum class Start (val strafe: Float, val forward: Float, val instructions: String, val targetTag: String) {
        RED_NEAR( 0f,0f, "Place front of robot against the goal, centered", "RedTarget"),
        RED_FAR(  0f,0f, "Place back of robot against the wall on outermost corner of far launch line", "RedTarget"),
        BLUE_NEAR(0f,0f, "Place front of robot against the goal, centered", "BlueTarget"),
        BLUE_FAR( 0f,0f, "Place back of robot against the wall on outermost corner of far launch line", "BlueTarget"),
        NONE(     0f,0f, "NO POSITION SELECTED", "")
    }

    var startingPosition = Start.NONE
    val artifactOrder = listOf(
        Artifact.GREEN,
        Artifact.PURPLE,
        Artifact.PURPLE
    )

    lateinit var camera: OV9281
    var currentTagBearing = 0.0
    var currentTagDistance = 0.0

    override fun runOpMode() {
        val drivetrain = Drivetrain(this)
        val transfer = Transfer(this)
        val shooter = Shooter(this)
        val spindexer = Spindexer(this)
        val turret = Turret(this)
        camera = OV9281(this, 4, 6)
        val timer = ElapsedTime()

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML)
        telemetry.addData("Drivetrain", "initialized")
        telemetry.update()


        var confirmed = false
        var preloadIndex = 0
        while (opModeInInit()) {
            if (!confirmed) {
                telemetry.addData("Drivetrain", "initialized")
                telemetry.addLine("Press CIRCLE   for blue alliance far")
                telemetry.addLine("Press CROSS    for red  alliance far")
                telemetry.addLine("Press SQUARE   for red  alliance near")
                telemetry.addLine("Press TRIANGLE for blue alliance near")
                telemetry.addLine()
                telemetry.addData("Selected position", "<strong>%s</strong>", startingPosition.name)
                telemetry.addData("<strong>INSTRUCTIONS</strong>", startingPosition.instructions)
                telemetry.addLine("Press TOUCHPAD to confirm selection")

                startingPosition = if (gamepad1.crossWasPressed()) {
                    Start.RED_FAR
                } else if (gamepad1.triangleWasPressed()) {
                    Start.BLUE_NEAR
                } else if (gamepad1.squareWasPressed()) {
                    Start.RED_NEAR
                } else if (gamepad1.circleWasPressed()) {
                    Start.BLUE_FAR
                } else {
                    startingPosition
                }

                if (gamepad1.touchpadWasPressed()) {
                    gamepad1.rumble(100)
                    confirmed = true
                }

            } else if (preloadIndex <= 2) {
                // once position is confirmed, go ahead and preload
                telemetry.addData("Starting position", "<strong>%s</strong>", startingPosition.name)
                telemetry.addLine()
                telemetry.addData("PRELOAD Configuration", artifactOrder.joinToString("") { it.firstLetter().toString() })
                telemetry.addData("Current Preload", artifactOrder[preloadIndex].name)
                telemetry.addData("Current Slot", spindexer.statesToSlotsMap.getValue(spindexer.state))
                telemetry.addLine()
                telemetry.addLine("Put ${artifactOrder[preloadIndex].name} artifact in current slot (${spindexer.statesToSlotsMap.getValue(spindexer.state)})")
                telemetry.addLine("Press CROSS once artifact is in place.")

                if (gamepad1.crossWasPressed()) {
                    spindexer.recordIntake(artifactOrder[preloadIndex])
                    preloadIndex++
                    spindexer.toNextIntakePosition()
                    gamepad1.rumble(300)
                }
            } else {
                telemetry.addLine("Configuration complete.")
                telemetry.addLine("Press START to run Autonomous.")
            }

            telemetry.update()
        }

        waitForStart()
        timer.reset()


        // if you are near, back up for like a second or so
        if (startingPosition == Start.RED_NEAR || startingPosition == Start.BLUE_NEAR) {
            drivetrain.setDrivetrainPowers(drivetrain.calculateDrivetrainPowers(0f, -0.5f, 0f))
            // wait for driveTime milliseconds
            while (timer.milliseconds() <= backupTime && opModeIsActive()) {
                telemetry.addData("Backing Up", "...")
                telemetry.update()
            }
            drivetrain.setDrivetrainPowers(drivetrain.calculateDrivetrainPowers(0f, 0f, 0f))
            timer.reset()

            // then turn right or left
            drivetrain.setDrivetrainPowers(drivetrain.calculateDrivetrainPowers(0f, 0f, if(startingPosition == Start.RED_NEAR) 0.5f else -0.5f))
            while (timer.milliseconds() <= rotateTime && opModeIsActive()) {
                telemetry.addData("Rotating", "...")
                telemetry.update()
            }
            drivetrain.setDrivetrainPowers(drivetrain.calculateDrivetrainPowers(0f, 0f, 0f))
            timer.reset()
        }

        // ok now get the motif
        while (spindexer.motifPattern == MotifPattern.NONE && opModeIsActive()) {
            spindexer.motifPattern = camera.getMotif()
        }

        spindexer.toMotifOuttakePosition()

        // turn back
        if (startingPosition == Start.RED_NEAR || startingPosition == Start.BLUE_NEAR) {
            // then turn right or left
            drivetrain.setDrivetrainPowers(drivetrain.calculateDrivetrainPowers(0f, 0f, if(startingPosition == Start.RED_NEAR) -0.5f else 0.5f))
            while (timer.milliseconds() <= rotateTime && opModeIsActive()) {
                spindexer.periodic()
                telemetry.addData("Rotating", "...")
                telemetry.update()
            }
            drivetrain.setDrivetrainPowers(drivetrain.calculateDrivetrainPowers(0f, 0f, 0f))
            timer.reset()
        }

        // ok so now we can start shooting
        var doneShooting = false
        var justFired = false

        while (!doneShooting && opModeIsActive()) {
            // update pids and get new camera readings
            updateCamera(startingPosition.targetTag)
            shooter.calculateTargetState(currentTagDistance)
            turret.periodic(currentTagBearing)
            shooter.periodic()
            spindexer.periodic()
            transfer.periodic()

            // if aimed
            if (turret.atSetPoint()) {
                // if we should wait for the transfer + flywheel to reset
                if (justFired) {
                    // wait until the ball is clear until you move the spindexer
                    if (transfer.atSetPoint()) {
                        justFired = false

                        // if there are still balls to shoot, cycle
                        if (!spindexer.isEmpty) {
                            spindexer.toNextOuttakePosition()
                        }
                    }
                    // if the flywheel is spun up
                } else if (shooter.atSetPoint() && spindexer.atSetPoint()) {
                    // shoot, record shoot, let it recover
                    transfer.transferArtifact()
                    spindexer.recordOuttake()
                    justFired = true
                }

                // if the balls are all shot and transfer has returned to rest
                if (spindexer.isEmpty && !justFired) {
                    doneShooting = true
                }
            }

            telemetry.addData("Just fired", justFired)
            telemetry.addLine("-------------")
            telemetry.addData("Shooter at set point", shooter.atSetPoint())
            telemetry.addData("transfer at set point", transfer.atSetPoint())
            telemetry.addData("Spindexer at set point", spindexer.atSetPoint())
            telemetry.addData("Turret at set point", turret.atSetPoint())
            telemetry.addLine("-------------")
            telemetry.addData("Distance to goal", currentTagDistance)
            telemetry.addData("Bearing to goal", currentTagBearing)
            telemetry.addData("Target tag", startingPosition.targetTag)
            telemetry.addData("Starting position", startingPosition.name)
            telemetry.addLine("-------------")
            telemetry.addData("Current Artifact assortment", spindexer.getArtifactString())
            telemetry.addData("Current spindexer position", spindexer.state.name)

            telemetry.update()
        }

        // ok shooting over
        // now move off / out of zones

        drivetrain.setDrivetrainPowers(drivetrain.calculateDrivetrainPowers(if(startingPosition == Start.RED_NEAR || startingPosition == Start.RED_FAR) 0.5f else -0.5f, 0f,  0f))
        while (timer.milliseconds() <= offLineTime && opModeIsActive()) {
            telemetry.addData("Leaving", "...")
            telemetry.update()
        }
        drivetrain.setDrivetrainPowers(drivetrain.calculateDrivetrainPowers(0f, 0f, 0f))
        timer.reset()
    }

    fun updateCamera(targetTag: String) {
        val currentDetections = camera.aprilTag.detections

        if (!currentDetections.isEmpty()) {
            telemetry.addData("Detected april tags", currentDetections.size)

            for (detection in currentDetections) {
                if (detection.metadata != null) {
                    telemetry.addData("TAG NAME", detection.metadata.name)

                    if (detection.metadata.name.contains(targetTag)) {
                        currentTagDistance = detection.ftcPose.range
                        currentTagBearing = -detection.ftcPose.bearing
                    }
                } else {
                    telemetry.addData("Current tag", "NO metadata")
                }
            }
        } else { // no detections
            telemetry.addData("Detected april tags", 0)
        }
    }
}
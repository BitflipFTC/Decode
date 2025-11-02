package org.firstinspires.ftc.teamcode.opmodes

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.hardware.Intake
import org.firstinspires.ftc.teamcode.hardware.OV9281
import org.firstinspires.ftc.teamcode.hardware.Shooter
import org.firstinspires.ftc.teamcode.hardware.Spindexer
import org.firstinspires.ftc.teamcode.hardware.Transfer
import org.firstinspires.ftc.teamcode.hardware.Turret
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.util.Artifact


class PedroPathing12RedAuto: OpMode() {
    private lateinit var follower: Follower
    private lateinit var pathTimer: Timer
    private lateinit var opmodeTimer: Timer

    lateinit var spindexer: Spindexer
    lateinit var intake: Intake
    lateinit var turret: Turret
    lateinit var camera: OV9281
    lateinit var shooter: Shooter
    lateinit var transfer: Transfer

    private var pathState = -1
    private var currentTagBearing: Double = 0.0
    private val targetTagBearing: Double = 0.0
    private var distanceToGoal: Double = -1.0

    var justFired = false

    override fun init() {
        pathTimer = Timer()
        opmodeTimer = Timer()
        opmodeTimer.resetTimer()

        spindexer = Spindexer(this)
        intake = Intake(this)
        turret = Turret(this)
        shooter = Shooter(this)
        transfer = Transfer(this)
        camera = OV9281(this, 4, 7)

        follower = Constants.createFollower(hardwareMap)
        buildPaths()
        follower.setStartingPose(startPose)
    }

    override fun loop() {
        follower.update()    // update pathing
        updateCamera()       // update tag position + distance to tag
        autonomousUpdate()   // FSM control

        spindexer.periodic() // rotate spindexer as needed
        intake.periodic()    // sets motor power to target power
//        turret.periodic(currentTagPosition)
        shooter.periodic()   // runs flywheel velocity pid + holds servo position
        transfer.periodic()  // runs transfer position pid as needed

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.pose.x);
        telemetry.addData("y", follower.pose.y);
        telemetry.addData("heading", follower.pose.heading);
        telemetry.addData("Current Red Goal Position", currentTagBearing)
        telemetry.addData("Target Red Goal Position", targetTagBearing)
        telemetry.addData("Distance to goal (in)", distanceToGoal)

        telemetry.update();
    }

    override fun start() {
        opmodeTimer.resetTimer()
        spindexer.motifPattern = camera.getMotif()

        intake.intake()
        spindexer.state = Spindexer.States.INTAKE_ZERO

        spindexer.recordIntake(Artifact.GREEN, 0)
        spindexer.recordIntake(Artifact.PURPLE, 1)
        spindexer.recordIntake(Artifact.PURPLE, 2)

        shooter.calculateTargetState(distanceToGoal)
        setPathState(0)
    }

    val horizontalIntakeStart = 100.0
    val horizontalIntakeEnd   = 125.0

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
        scorePreload = follower.pathBuilder()
            .addPath(
                BezierLine(
                    startPose,
                    scorePose
                )
            )
            .setLinearHeadingInterpolation(startPose.heading, scorePose.heading)
            .build()

        intake1 = follower.pathBuilder()
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

        score1 = follower.pathBuilder()
            .addPath(
                BezierLine(endIntake1, scorePose)
            )
            .setLinearHeadingInterpolation(endIntake1.heading, scorePose.heading)
            .build()
        
        // shoot + aim

        intake2 = follower.pathBuilder()
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

        score2 = follower.pathBuilder()
            .addPath(
                BezierLine(endIntake2, scorePose)
            )
            .setLinearHeadingInterpolation(endIntake2.heading, scorePose.heading)
            .build()

        intake3 = follower.pathBuilder()
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

        score3 = follower.pathBuilder()
            .addPath(
                BezierLine(endIntake3, scorePose)
            )
            .setLinearHeadingInterpolation(endIntake3.heading, scorePose.heading)
            .build()

        park = follower.pathBuilder()
            .addPath(
                BezierLine(scorePose, parkPose)
            )
            .setLinearHeadingInterpolation(scorePose.heading, parkPose.heading)
            .build()
    }

    fun autonomousUpdate() {
        when (pathState) {
            0 -> {
                follower.followPath(scorePreload, true)
                spindexer.toMotifOuttakePosition()
                setPathState(1)
            }

            1 -> {
                if (!follower.isBusy) {
                    shooter.calculateTargetState(distanceToGoal)
                    turret.bearing = currentTagBearing
                    turret.periodic()

                    // if aimed
                    if (turret.atSetPoint()) {
                        // if we should wait for the transfer + flywheel to reset
                        if (justFired) {
                            if (transfer.atSetPoint()) {
                                justFired = false

                                // if there are still balls to shoot, cycle
                                if (spindexer.getArtifactString() != "NNN") {
                                    spindexer.toNextOuttakePosition()
                                }
                            }
                            // if the flywheel is right
                        } else if (shooter.atSetPoint()) {
                            transfer.transferArtifact()
                            spindexer.recordOuttake()
                            justFired = true
                        }

                        // if the balls are all shot and transfer has returned to rest
                        if (spindexer.getArtifactString() == "NNN" && !justFired) {
                            spindexer.toNextIntakePosition() // zero
                            follower.followPath(intake1)
                            setPathState(2)
                        }
                    }
                }
            }

            2 -> {
                // add some way to detect intake, then rotate spindexer toNextIntakePosition()
            }
        }
    }

    fun setPathState (state: Int) {
        pathState = state
        pathTimer.resetTimer()
    }

    fun updateCamera() {
        /*
            public static AprilTagLibrary getDecodeTagLibrary(){
                return new AprilTagLibrary.Builder()
                        .addTag(20, "BlueTarget",
                                6.5, new VectorF(-58.3727f, -55.6425f, 29.5f), DistanceUnit.INCH,
                                new Quaternion(0.2182149f, -0.2182149f, -0.6725937f, 0.6725937f, 0))
                        .addTag(21, "Obelisk_GPP",
                                6.5, DistanceUnit.INCH)
                        .addTag(22, "Obelisk_PGP",
                                6.5, DistanceUnit.INCH)
                        .addTag(23, "Obelisk_PPG",
                                6.5, DistanceUnit.INCH)
                        .addTag(24, "RedTarget",
                                6.5, new VectorF(-58.3727f, 55.6425f, 29.5f), DistanceUnit.INCH,
                                new Quaternion(0.6725937f, -0.6725937f, -0.2182149f, 0.2182149f, 0))
                        .build();
            }
         */
        val currentDetections = camera.aprilTag.detections

        if (!currentDetections.isEmpty()) {
            telemetry.addData("Detected april tags", currentDetections.size)

            val detection = currentDetections[0]
            if (detection.metadata != null) {
                telemetry.addData("TAG NAME", detection.metadata.name)

                if (detection.metadata.name.contains("RedTarget")) {
                    distanceToGoal = detection.ftcPose.range
                    currentTagBearing = -detection.ftcPose.bearing
                }
            } else {
                telemetry.addData("Current tag", "NO metadata")
            }
        } else { // no detections
            telemetry.addData("Detected april tags", 0)
        }

    }
}
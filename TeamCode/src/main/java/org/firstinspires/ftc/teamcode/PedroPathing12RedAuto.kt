package org.firstinspires.ftc.teamcode

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.hardware.Intake
import org.firstinspires.ftc.teamcode.hardware.OV9281
import org.firstinspires.ftc.teamcode.hardware.Shooter
import org.firstinspires.ftc.teamcode.hardware.Spindexer
import org.firstinspires.ftc.teamcode.hardware.Transfer
import org.firstinspires.ftc.teamcode.hardware.Turret
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.util.Artifact
import org.firstinspires.ftc.teamcode.util.InterpolatedLookupTable
import kotlin.math.pow
import kotlin.math.sqrt


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

    private val lookupTable = InterpolatedLookupTable(
            doubleArrayOf(),
            doubleArrayOf(),
            doubleArrayOf(),
        doubleArrayOf()
    )

    private var pathState = -1
    private var currentTagPosition: Double = 320.0
    private val targetTagPosition: Double = 320.0
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
        telemetry.addData("Current Red Goal Position", currentTagPosition)
        telemetry.addData("Target Red Goal Position", targetTagPosition)
        telemetry.addData("Distance to goal (in)", distanceToGoal)

        telemetry.update();
    }

    override fun start() {
        opmodeTimer.resetTimer()
        spindexer.motifPattern = getMotif()

        intake.on()
        spindexer.setTargetState(Spindexer.States.INTAKE_ZERO)

        spindexer.recordIntake(Artifact.GREEN, 0)
        spindexer.recordIntake(Artifact.PURPLE, 1)
        spindexer.recordIntake(Artifact.PURPLE, 2)

        shooter.setState(lookupTable.calculate(distanceToGoal))
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
                    shooter.setState(lookupTable.calculate(distanceToGoal))

                    if (justFired) {
                        if (transfer.atSetPoint()) {
                            justFired = false

                            if (spindexer.getArtifactString() != "NNN") {
                                spindexer.toNextOuttakePosition()
                            }
                        }
                    } else if (shooter.atSetPoint()) {
                        transfer.transferArtifact()
                        spindexer.recordOuttake()
                        justFired = true
                    }

                    if (spindexer.getArtifactString() == "NNN" && !justFired) {
                        justFired = false
                        spindexer.toNextIntakePosition() // zero
                        follower.followPath(intake1)
                        setPathState(2)
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

                    // DISTANCE CALCULATIONS
                    val tagPos = Pose2D(
                        DistanceUnit.INCH,
                        detection.metadata.fieldPosition.get(0).toDouble(),
                        detection.metadata.fieldPosition.get(1).toDouble(),
                        AngleUnit.DEGREES,
                        detection.metadata.fieldOrientation.toOrientation(
                            AxesReference.EXTRINSIC,
                            AxesOrder.XYZ,
                            AngleUnit.DEGREES
                        ).secondAngle.toDouble()
                    ) // given north is 0, clockwise

                    val robotPos = Pose2D(
                        detection.robotPose.getPosition().unit,
                        detection.robotPose.getPosition().x,
                        detection.robotPose.getPosition().y,
                        AngleUnit.DEGREES,
                        detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)
                    )

                    distanceToGoal = sqrt(
                        (tagPos.getX(DistanceUnit.INCH) - robotPos.getX(DistanceUnit.INCH)).pow(
                            2.0
                        ) + (tagPos.getY(DistanceUnit.INCH) - robotPos.getY(DistanceUnit.INCH)).pow(
                            2.0
                        )
                    )
                    // END DISTANCE CALCS

                    currentTagPosition = detection.center.x
                }
            } else {
                telemetry.addData("Current tag", "NO metadata")
            }
        } else { // no detections
            telemetry.addData("Detected april tags", 0)
        }

    }

    fun getMotif(): Spindexer.MotifPattern {
        val currentDetections = camera.aprilTag.detections
        var pattern: Spindexer.MotifPattern = Spindexer.MotifPattern.NONE
        for (detection in currentDetections) {
            when (detection.metadata.name) {
                "Obelisk_GPP" -> pattern = Spindexer.MotifPattern.GPP
                "Obelisk_PGP" -> pattern = Spindexer.MotifPattern.PGP
                "Obelisk_PPG" -> pattern = Spindexer.MotifPattern.PPG
                else          -> {}
            }
        }

        return pattern
    }
}
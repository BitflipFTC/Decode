package org.firstinspires.ftc.teamcode.test

import android.util.Log
import android.util.Size
import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.pow
import kotlin.math.sqrt

@Configurable
@TeleOp(name = "Concept: AprilTag Driver", group = "Concept")
class AprilTagDriver : LinearOpMode() {
    companion object {
        @JvmField var kP: Double = 0.0025
        @JvmField var kI: Double = 0.0
        @JvmField var kD: Double = 0.0
        @JvmField var min : Double = -10.0
        @JvmField var max : Double = 10.0
        @JvmField var driveSpeed: Double = 0.5
    }
    private val frontLeft  by lazy { hardwareMap["frontleft"]  as DcMotorEx }
    private val frontRight by lazy { hardwareMap["frontright"] as DcMotorEx }
    private val backLeft   by lazy { hardwareMap["backleft"]   as DcMotorEx }
    private val backRight  by lazy { hardwareMap["backright"]  as DcMotorEx }

    private val imu        by lazy { hardwareMap["imu"] as IMU }
    private var currentTagPos = 320.0
    private var lastTagPos = 320.0
    private var targetTagPos = 320.0

    val timer = ElapsedTime()

    lateinit var aprilTag : AprilTagProcessor
    lateinit var visionPortal : VisionPortal

    val distArr = ArrayList<Double>()
    var filteredDist = 0.0

//    val controller = PIDController(
//        kP,kI,kD, 0.0, 0.0, max,
//        min)
    override fun runOpMode() {
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)

//        frontLeft.direction = DcMotorSimple.Direction.REVERSE
//        backLeft.direction  = DcMotorSimple.Direction.REVERSE
//
//        frontLeft.mode  = DcMotor.RunMode.STOP_AND_RESET_ENCODER
//        frontRight.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
//        backLeft.mode   = DcMotor.RunMode.STOP_AND_RESET_ENCODER
//        backRight.mode  = DcMotor.RunMode.STOP_AND_RESET_ENCODER
//
//        frontLeft.zeroPowerBehavior  = DcMotor.ZeroPowerBehavior.BRAKE
//        frontRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
//        backLeft.zeroPowerBehavior   = DcMotor.ZeroPowerBehavior.BRAKE
//        backRight.zeroPowerBehavior  = DcMotor.ZeroPowerBehavior.BRAKE
//
//        frontLeft.mode  = DcMotor.RunMode.RUN_WITHOUT_ENCODER
//        frontRight.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
//        backLeft.mode   = DcMotor.RunMode.RUN_WITHOUT_ENCODER
//        backRight.mode  = DcMotor.RunMode.RUN_WITHOUT_ENCODER

//        controller.setPointTolerance = 1.toDouble()

//        imu.initialize(IMU.Parameters(RevHubOrientationOnRobot(
//            RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
//            RevHubOrientationOnRobot.UsbFacingDirection.UP)))

        initAprilTag()

        // bulk caching
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        allHubs.forEach { hub -> hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        waitForStart()
        timer.reset()
        var lastState = visionPortal.cameraState
        var startTime = 0.toDouble()
        val samplesTimer = ElapsedTime()

        while (opModeIsActive()) {
            // more bulk caching
            allHubs.forEach { hub -> hub.clearBulkCache() }

//            controller.setCoeffs(kP,kI,kD)
//            controller.setIntegrationBounds(min, max)
//            val filtered = currentTagPos * 0.25 + lastTagPos * (1-0.25)
//            lastTagPos = currentTagPos
//            val pidOutput = controller.calculate(filtered,targetTagPos)

//            val x : Double = gamepad1.left_stick_x.toDouble()
//            val y : Double = -gamepad1.left_stick_y.toDouble()
//            val rx : Double = gamepad1.right_stick_x.toDouble()

//            var rot = 0.0
//            if (abs(pidOutput) >= 0.12)
//                rot = -pidOutput

//            val frontLeftPower = y + x + rot
//            val frontRightPower = y - x - rot
//            val backLeftPower = y - x + rot
//            val backRightPower = y + x - rot
            if (gamepad1.circleWasPressed()) {
                timer.reset()
                visionPortal.resumeStreaming()
                gamepad1.rumble(0.0, 1.0, 500)
            }

            if (visionPortal.cameraState == VisionPortal.CameraState.STREAMING) {
                if (lastState == VisionPortal.CameraState.STARTING_STREAM) {
                    startTime = timer.milliseconds()
                    samplesTimer.reset()
                }

                if (samplesTimer.seconds() >= 5.0){
                    Log.d("CAMERA", "NO SAMPLES FOUND. CLOSING CAMERA")
                    visionPortal.stopStreaming()
                } else if (distArr.size >= 10) {
                    Log.d("CAMERA", "10 SAMPLES RECEIVED. YAY! CLOSING CAMERA")
                    visionPortal.stopStreaming()
                } else {
                    processVision()
                }
            }

            var allInArr = 0.0
            distArr.forEach { dist -> allInArr += dist}
            filteredDist = allInArr / distArr.size


            lastState = visionPortal.cameraState

//            frontLeft.power  = frontLeftPower  * driveSpeed
//            frontRight.power = frontRightPower * driveSpeed
//            backLeft.power   = backLeftPower   * driveSpeed
//            backRight.power  = backRightPower  * driveSpeed

            telemetry.addData("Starting time (ms)", startTime)
            telemetry.addData("Filtered Dist", filteredDist)
            telemetry.addData("Sampling Time", samplesTimer.seconds())

//            telemetry.addData("PID Output", pidOutput)
//            telemetry.addLine("---------------------------------------")
//            telemetry.addData("Front Left Wheel", frontLeftPower)
//            telemetry.addData("Front Right Wheel", frontRightPower)
//            telemetry.addData("Back Left Wheel", backLeftPower)
//            telemetry.addData("Back Right Wheel", backRightPower)
//            telemetry.addLine("---------------------------------------")
//            telemetry.addData("Left_X",x)
//            telemetry.addData("Left_Y",y)
//            telemetry.addData("RightX",rx)
//            telemetry.addLine("---------------------------------------")
//            telemetry.addData("PID Controller variables", "")
//            telemetry.addData("PID kP", controller.kP)
//            telemetry.addData("PID kI", controller.kI)
//            telemetry.addData("PID kD", controller.kD)
//            telemetry.addData("PID Target Position", controller.setpoint)
//            telemetry.addData("PID Current Position", controller.processVariable)
//            telemetry.addData("Last position", lastTagPos)
//            telemetry.addData("PID Error", controller.error)
//            telemetry.addData("PID Velocity error", controller.velError)
//            telemetry.addData("PID Last Error", controller.lastError)
//            telemetry.addData("PID SetPoint Tolerance", controller.setPointTolerance)
//            telemetry.addData("PID Loop time", controller.timePeriod)
//            telemetry.addData("PID Total Error", controller.totalError)
//            telemetry.addData("PID At SetPoint", controller.atSetPoint())

            telemetry.update()
        }
        visionPortal.close()

    }

    private fun initAprilTag() {
        aprilTag = AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawTagOutline(true)
            .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
            .build()
        aprilTag.setDecimation(3f)

        // Create the vision portal the easy way.
        val builder = VisionPortal.Builder()

        builder.setCamera(hardwareMap.get<WebcamName?>(WebcamName::class.java, "camera"))

        builder.setCameraResolution(Size(640, 480))
        builder.enableLiveView(true)
        builder.addProcessor(aprilTag)
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
        visionPortal = builder.build()
        visionPortal.stopStreaming()
    }

    private fun processVision() {
        val currentDetections = aprilTag.detections

        if (currentDetections.isEmpty()) {
            currentTagPos = targetTagPos
        } else {
            telemetry.addData("Detected april tags", currentDetections.size)
            currentDetections.forEach { detection -> {
                if (!detection.metadata.name.contains("Obelisk")) {
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

                    val dist = sqrt(
                        (tagPos.getX(DistanceUnit.INCH) - robotPos.getX(DistanceUnit.INCH)).pow(
                            2.0
                        ) + (tagPos.getY(DistanceUnit.INCH) - robotPos.getY(DistanceUnit.INCH)).pow(
                            2.0
                        )
                    )
                    distArr.add(dist)
                } else {
                    telemetry.addData("TAG NAME", detection.metadata.name)
                }

                currentTagPos = detection.center.x
            }
            }
        }
    }
}
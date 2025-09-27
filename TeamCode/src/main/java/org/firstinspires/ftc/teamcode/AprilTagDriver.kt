package org.firstinspires.ftc.teamcode

import android.util.Size
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.util.AprilTagDriverPID
import org.firstinspires.ftc.teamcode.util.PIDController
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.openftc.easyopencv.OpenCvWebcam
import kotlin.math.abs

@Configurable
@TeleOp(name = "Concept: AprilTag Driver", group = "Concept")
class AprilTagDriver : LinearOpMode() {
    private val frontLeft  by lazy { hardwareMap["frontleft"]  as DcMotorEx }
    private val frontRight by lazy { hardwareMap["frontright"] as DcMotorEx }
    private val backLeft   by lazy { hardwareMap["backleft"]   as DcMotorEx }
    private val backRight  by lazy { hardwareMap["backright"]  as DcMotorEx }

    private val imu        by lazy { hardwareMap["imu"] as IMU }
    private var currentTagPos = 320.0
    private var lastTagPos = 320.0
    private var targetTagPos = 320.0

    lateinit var aprilTag : AprilTagProcessor
    lateinit var visionPortal : VisionPortal

    companion object {
        @JvmField
        var driveSpeed: Double = 0.5
    }

    val controller = PIDController(
        AprilTagDriverPID.p,AprilTagDriverPID.i,AprilTagDriverPID.d, 0.0, AprilTagDriverPID.max,
        AprilTagDriverPID.min)
    override fun runOpMode() {
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry, FtcDashboard.getInstance().telemetry)

        frontLeft.direction = DcMotorSimple.Direction.REVERSE
        backLeft.direction  = DcMotorSimple.Direction.REVERSE

        frontLeft.mode  = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        frontRight.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        backLeft.mode   = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        backRight.mode  = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        frontLeft.zeroPowerBehavior  = DcMotor.ZeroPowerBehavior.BRAKE
        frontRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        backLeft.zeroPowerBehavior   = DcMotor.ZeroPowerBehavior.BRAKE
        backRight.zeroPowerBehavior  = DcMotor.ZeroPowerBehavior.BRAKE

        frontLeft.mode  = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        frontRight.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        backLeft.mode   = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        backRight.mode  = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        controller.setPointTolerance = 1.toDouble()

        imu.initialize(IMU.Parameters(RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
            RevHubOrientationOnRobot.UsbFacingDirection.UP)))

        initAprilTag()

        // bulk caching
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        allHubs.forEach { hub -> hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        waitForStart()

        while (opModeIsActive()) {
            // more bulk caching
            allHubs.forEach { hub -> hub.clearBulkCache() }

            processVision()
            controller.setCoeffs(AprilTagDriverPID.p,AprilTagDriverPID.i,AprilTagDriverPID.d)
            controller.setIntegrationBounds(AprilTagDriverPID.min, AprilTagDriverPID.max)
            val filtered = currentTagPos * 0.25 + lastTagPos * (1-0.25)
            lastTagPos = currentTagPos
            val pidOutput = controller.calculate(filtered,targetTagPos)

            val x : Double = gamepad1.left_stick_x.toDouble()
            val y : Double = -gamepad1.left_stick_y.toDouble()
            val rx : Double = gamepad1.right_stick_x.toDouble()

            var rot = 0.0
            if (abs(pidOutput) >= 0.12)
                rot = -pidOutput

            val frontLeftPower = y + x + rot
            val frontRightPower = y - x - rot
            val backLeftPower = y - x + rot
            val backRightPower = y + x - rot

            frontLeft.power  = frontLeftPower  * driveSpeed
            frontRight.power = frontRightPower * driveSpeed
            backLeft.power   = backLeftPower   * driveSpeed
            backRight.power  = backRightPower  * driveSpeed

            telemetry.addData("PID Output", pidOutput)
            telemetry.addLine("---------------------------------------")
            telemetry.addData("Front Left Wheel", frontLeftPower)
            telemetry.addData("Front Right Wheel", frontRightPower)
            telemetry.addData("Back Left Wheel", backLeftPower)
            telemetry.addData("Back Right Wheel", backRightPower)
            telemetry.addLine("---------------------------------------")
            telemetry.addData("Left_X",x)
            telemetry.addData("Left_Y",y)
            telemetry.addData("RightX",rx)
            telemetry.addLine("---------------------------------------")
            telemetry.addData("PID Controller variables", "")
            telemetry.addData("PID kP", controller.kP)
            telemetry.addData("PID kI", controller.kI)
            telemetry.addData("PID kD", controller.kD)
            telemetry.addData("PID Target Position", controller.targetPosition)
            telemetry.addData("PID Current Position", controller.currentPosition)
            telemetry.addData("Last position", lastTagPos)
            telemetry.addData("PID Error", controller.error)
            telemetry.addData("PID Velocity error", controller.velError)
            telemetry.addData("PID Last Error", controller.lastError)
            telemetry.addData("PID SetPoint Tolerance", controller.setPointTolerance)
            telemetry.addData("PID Loop time", controller.timePeriod)
            telemetry.addData("PID Total Error", controller.totalError)
            telemetry.addData("PID At SetPoint", controller.atSetPoint())

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
    }

    private fun processVision() {
        val currentDetections = aprilTag.detections

        if (currentDetections.isEmpty()) {
            currentTagPos = targetTagPos;
        } else {
            telemetry.addData("Detected april tags", currentDetections.size)
            currentDetections.forEach { detection -> currentTagPos = detection.center.x }
        }
    }
}
package org.firstinspires.ftc.teamcode.test

import android.annotation.SuppressLint
import android.graphics.Color
import android.util.Size
import com.acmerobotics.dashboard.FtcDashboard
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName
import org.firstinspires.ftc.teamcode.util.ArtifactTrackAutoPID.kD
import org.firstinspires.ftc.teamcode.util.ArtifactTrackAutoPID.kI
import org.firstinspires.ftc.teamcode.util.ArtifactTrackAutoPID.kP
import org.firstinspires.ftc.teamcode.util.ArtifactTrackAutoPID.kS
import org.firstinspires.ftc.teamcode.util.ArtifactTrackAutoPID.kV
import org.firstinspires.ftc.teamcode.util.ArtifactTrackAutoPID.maxCirc
import org.firstinspires.ftc.teamcode.util.ArtifactTrackAutoPID.maxIntegral
import org.firstinspires.ftc.teamcode.util.ArtifactTrackAutoPID.maxSize
import org.firstinspires.ftc.teamcode.util.ArtifactTrackAutoPID.minCirc
import org.firstinspires.ftc.teamcode.util.ArtifactTrackAutoPID.minIntegral
import org.firstinspires.ftc.teamcode.util.ArtifactTrackAutoPID.minSize
import org.firstinspires.ftc.teamcode.util.PIDController
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor
import org.firstinspires.ftc.vision.opencv.ColorRange
import org.firstinspires.ftc.vision.opencv.ImageRegion
import java.util.concurrent.CopyOnWriteArrayList
import java.util.concurrent.CopyOnWriteArraySet

@TeleOp(name = "Test: Artifact Track", group = "Test")
class ArtifactTrackAuto : LinearOpMode() {
    private val frontLeft  by lazy { hardwareMap["frontleft"]  as DcMotorEx }
    private val frontRight by lazy { hardwareMap["frontright"] as DcMotorEx }
    private val backLeft   by lazy { hardwareMap["backleft"]   as DcMotorEx }
    private val backRight  by lazy { hardwareMap["backright"]  as DcMotorEx }
    private val imu        by lazy { hardwareMap["imu"] as IMU }
    lateinit var visionPortal : VisionPortal
    private var artifactLocator = ArtifactCirclePipeline()
    private val aCamera by lazy { hardwareMap["aCamera"] as CameraName }

    private val controller = PIDController(kP,kI,kD,kV,kS,maxIntegral,minIntegral)

    @SuppressLint("DefaultLocale")
    override fun runOpMode() {
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry,telemetry, FtcDashboard.getInstance().telemetry)
        initArtifactCamera()
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE)
        telemetry.captionValueSeparator = ""

        waitForStart()

        while (opModeIsActive() || opModeInInit()) {
            val foundArtifacts = ArrayList<ArtifactCirclePipeline.Artifact>(artifactLocator.foundArtifacts)
//            telemetry.addData("stuff", foundArtifacts.first())
            telemetry.addLine( "x     y     radius     color ")
            for (artifact in foundArtifacts) {
                telemetry.addData("", "%05.1f %05.1f %05.2f %s",
                    artifact.center.x, artifact.center.y, artifact.radius, artifact.color.name)
            }
            telemetry.update()
        }
    }

    fun initArtifactCamera () {
        visionPortal = VisionPortal.Builder()
            .setCamera(aCamera)
            .addProcessor(artifactLocator)
            .setCameraResolution(Size(320, 240))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .enableLiveView(true)
            .setAutoStopLiveView(true)
            .build()
    }
}
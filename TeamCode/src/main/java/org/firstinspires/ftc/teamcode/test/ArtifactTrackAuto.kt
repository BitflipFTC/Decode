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

@TeleOp(name = "Test: Artifact Track", group = "Test")
class ArtifactTrackAuto : LinearOpMode() {
    private val frontLeft  by lazy { hardwareMap["frontleft"]  as DcMotorEx }
    private val frontRight by lazy { hardwareMap["frontright"] as DcMotorEx }
    private val backLeft   by lazy { hardwareMap["backleft"]   as DcMotorEx }
    private val backRight  by lazy { hardwareMap["backright"]  as DcMotorEx }
    private val imu        by lazy { hardwareMap["imu"] as IMU }
    private lateinit var visionPortal : VisionPortal
    private lateinit var colorLocator : ColorBlobLocatorProcessor
    private val aCamera by lazy { hardwareMap["aCamera"] as CameraName }

    private val controller = PIDController(kP,kI,kD,kV,kS,maxIntegral,minIntegral)

    @SuppressLint("DefaultLocale")
    override fun runOpMode() {
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry,telemetry, FtcDashboard.getInstance().telemetry)
        initArtifactCamera()

//        telemetry.msTransmissionInterval = 100 // Speed up telemetry updates for debugging.
//        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE)

        waitForStart()
        while (opModeIsActive()) {
            val blobs: List<ColorBlobLocatorProcessor.Blob> = colorLocator.blobs

            ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                minSize, maxSize, blobs
            )

            ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                minCirc, maxCirc, blobs
            )

            /*
             * The list of Blobs can be sorted using the same Blob attributes as listed above.
             * No more than one sort call should be made.  Sorting can use ascending or descending order.
             * Here is an example.:
             *   ColorBlobLocatorProcessor.Util.sortByCriteria(
             *      ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, blobs);
             */

            telemetry.addLine("Circularity Radius Center")

            // Display the Blob's circularity, and the size (radius) and center location of its circleFit.
            for (b in blobs) {
                val circleFit = b.circle
                telemetry.addLine(
                    String.format(
                        "%5.3f      %3d     (%3d,%3d)",
                        b.circularity,
                        circleFit.radius.toInt(),
                        circleFit.x.toInt(),
                        circleFit.y.toInt()
                    )
                )
            }

            telemetry.update()
        }
    }

    fun initArtifactCamera () {
        /* Build a "Color Locator" vision processor based on the ColorBlobLocatorProcessor class.
         * - Specify the color range you are looking for. Use a predefined color, or create your own
         *
         *   .setTargetColorRange(ColorRange.BLUE)     // use a predefined color match
         *     Available colors are: RED, BLUE, YELLOW, GREEN, ARTIFACT_GREEN, ARTIFACT_PURPLE
         *   .setTargetColorRange(new ColorRange(ColorSpace.YCrCb,  // or define your own color match
         *                                       new Scalar( 32, 176,  0),
         *                                       new Scalar(255, 255, 132)))
         *
         * - Focus the color locator by defining a RegionOfInterest (ROI) which you want to search.
         *     This can be the entire frame, or a sub-region defined using:
         *     1) standard image coordinates or 2) a normalized +/- 1.0 coordinate system.
         *     Use one form of the ImageRegion class to define the ROI.
         *       ImageRegion.entireFrame()
         *       ImageRegion.asImageCoordinates(50, 50,  150, 150)  100x100 pixels at upper left corner
         *       ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5)  50% width/height in center
         *
         * - Define which contours are included.
         *   You can get ALL the contours, ignore contours that are completely inside another contour.
         *     .setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)
         *     .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
         *     EXTERNAL_ONLY helps to avoid bright reflection spots from breaking up solid colors.
         *
         * - Turn the displays of contours ON or OFF.
         *     Turning these on helps debugging but takes up valuable CPU time.
         *        .setDrawContours(true)                Draws an outline of each contour.
         *        .setEnclosingCircleColor(int color)   Draws a circle around each contour. 0 to disable.
         *        .setBoxFitColor(int color)            Draws a rectangle around each contour. 0 to disable. ON by default.
         *
         *
         * - include any pre-processing of the image or mask before looking for Blobs.
         *     There are some extra processing you can include to improve the formation of blobs.
         *     Using these features requires an understanding of how they may effect the final
         *     blobs.  The "pixels" argument sets the NxN kernel size.
         *        .setBlurSize(int pixels)
         *        Blurring an image helps to provide a smooth color transition between objects,
         *        and smoother contours.  The higher the number, the more blurred the image becomes.
         *        Note:  Even "pixels" values will be incremented to satisfy the "odd number" requirement.
         *        Blurring too much may hide smaller features.  A size of 5 is good for a 320x240 image.
         *
         *     .setErodeSize(int pixels)
         *        Erosion removes floating pixels and thin lines so that only substantive objects remain.
         *        Erosion can grow holes inside regions, and also shrink objects.
         *        "pixels" in the range of 2-4 are suitable for low res images.
         *
         *     .setDilateSize(int pixels)
         *        Dilation makes objects and lines more visible by filling in small holes, and making
         *        filled shapes appear larger. Dilation is useful for joining broken parts of an
         *        object, such as when removing noise from an image.
         *        "pixels" in the range of 2-4 are suitable for low res images.
         *
         *        .setMorphOperationType(MorphOperationType morphOperationType)
         *        This defines the order in which the Erode/Dilate actions are performed.
         *        OPENING:    Will Erode and then Dilate which will make small noise blobs go away
         *        CLOSING:    Will Dilate and then Erode which will tend to fill in any small holes in blob edges.
         */
        colorLocator = ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(ColorRange.ARTIFACT_PURPLE) // Use a predefined color match
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
            .setRoi(ImageRegion.asUnityCenterCoordinates((-1).toDouble(), 1.0, 1.0, (-1).toDouble()))
            .setDrawContours(true) // Show contours on the Stream Preview
            .setBoxFitColor(0) // Disable the drawing of rectangles
            .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
            .setBlurSize(5) // Smooth the transitions between different colors in image
            // the following options have been added to fill in perimeter holes.

            .setDilateSize(15) // Expand blobs to fill any divots on the edges
            .setErodeSize(15) // Shrink blobs back to original size
            .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

            .build()

        visionPortal = VisionPortal.Builder()
            .setCamera(aCamera)
            .addProcessor(colorLocator)
            .setCameraResolution(Size(320, 240))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .enableLiveView(true)
            .setAutoStopLiveView(true)
            .build()
    }
}
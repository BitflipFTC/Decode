package org.firstinspires.ftc.teamcode.util;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

public class ColorSensorPipeline implements VisionProcessor {
    /*
     * The core values which define the location and size o1f the sample regions
     */
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(60,5);
    static final int REGION_WIDTH = 50;
    static final int REGION_HEIGHT = 50;

    /*
     * Points which actually define the sample region rectangles, derived from above values
     * Example of how points A and B work to define a rectangle
     *
     *   ------------------------------------
     *   | (0,0) Point A                    |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                  Point B (70,50) |
     *   ------------------------------------
     */

    private final Point region_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    private final Point region_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    private final Rect regionRect = new Rect(region_pointA, region_pointB);

    Mat HSV = new Mat();

    @Nullable
    private volatile Artifact artifact = null;

    private final Paint purplePaint = new Paint();
    private final Paint greenPaint = new Paint();
    private final Paint outlinePaint = new Paint();

    public ColorSensorPipeline () {
        purplePaint.setColor(Color.MAGENTA);
        purplePaint.setStyle(Paint.Style.FILL);
        purplePaint.setAlpha(120);
        greenPaint.setColor(Color.GREEN);
        greenPaint.setStyle(Paint.Style.FILL);
        greenPaint.setAlpha(120);
        outlinePaint.setColor(Color.BLUE);
        outlinePaint.setStyle(Paint.Style.STROKE);
        outlinePaint.setStrokeWidth(2f);
    }

    public static class HSV {
        private double hue, saturation, value;

        public synchronized void setValues (double[] newValues) {
            hue = newValues[0];
            saturation = newValues[1] ;
            value = newValues[2];
        }

        public synchronized double getHue() {
            return hue;
        }

        public synchronized double getSaturation() {
            return saturation;
        }

        public synchronized double getValue() {
            return value;
        }
    }
    public volatile HSV averages = new HSV();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        /*
         * Get the Cb channel of the input frame after conversion to YCrCb
         */
        Mat croppedFrame = frame.submat(regionRect);

        Imgproc.cvtColor(croppedFrame, HSV, Imgproc.COLOR_RGB2HSV);

        croppedFrame.release();

        /*
         * Compute the average pixel value of each submat region. We're
         * taking the average of a single channel buffer, so the value
         * we need is at index 0. We could have also taken the average
         * pixel value of the 3-channel image, and referenced the value
         * at index 2 here.
         */

        averages.setValues(Core.mean(HSV).val);

        if (averages.getValue() > 80 && averages.getHue() > 55) {
            if (averages.getHue() > 100) {
                artifact = Artifact.PURPLE;
            } else {
                artifact = Artifact.GREEN;
            }
        } else {
            artifact = null;
        }

        /*
         * Render the 'input' buffer to the viewport. But note this is not
         * simply rendering the raw camera feed, because we called functions
         * to add some annotations to this buffer earlier up.
         */
        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        android.graphics.Rect artifactRect = makeGraphicsRect(regionRect, scaleBmpPxToCanvasPx);

        Artifact local = artifact;
        if (local != null) {
            switch (local) {
                case GREEN:
                    canvas.drawRect(artifactRect, greenPaint);
                    break;
                case PURPLE:
                    canvas.drawRect(artifactRect, purplePaint);
                    break;
            }
        }

        canvas.drawRect(artifactRect, outlinePaint);
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    /**
     * Call this from the OpMode thread to obtain the latest analysis
     */
    @Nullable
    public Artifact getArtifact()
    {
        return artifact;
    }
}

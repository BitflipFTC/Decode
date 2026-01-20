package org.firstinspires.ftc.teamcode.util;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.util.ArrayList;
import java.util.Arrays;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorSensorPipeline implements VisionProcessor {
    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    boolean firstRun = true;
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        /*
         * Get the Cb channel of the input frame after conversion to YCrCb
         */
        splitInput(frame);

        if (firstRun) {
            region_H = H.submat(new Rect(region_pointA, region_pointB));
            region_S = S.submat(new Rect(region_pointA, region_pointB));
            region_V = V.submat(new Rect(region_pointA, region_pointB));
            firstRun = false;
        }

        /*
         * Compute the average pixel value of each submat region. We're
         * taking the average of a single channel buffer, so the value
         * we need is at index 0. We could have also taken the average
         * pixel value of the 3-channel image, and referenced the value
         * at index 2 here.
         */
        avgH = (int) Core.mean(region_H).val[0];
        avgS = (int) Core.mean(region_S).val[0];
        avgV = (int) Core.mean(region_V).val[0];

        if (avgS > 150) {
            if (avgH > 100) {
                artifact = Artifact.PURPLE;
            } else {
                artifact = Artifact.GREEN;
            }
        } else {
            artifact = null;
        }

        /*
         * Draw a rectangle showing sample region 1 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                frame, // Buffer to draw on
                region_pointA, // First point which defines the rectangle
                region_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        telemetry.addData("[Detected Artifact]", artifact);
        telemetry.addData("Average h", avgH);
        telemetry.addData("Average s", avgS);
        telemetry.addData("Average v", avgV);
        telemetry.update();

        /*
         * Render the 'input' buffer to the viewport. But note this is not
         * simply rendering the raw camera feed, because we called functions
         * to add some annotations to this buffer earlier up.
         */
        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint p = new Paint();
        p.setColor(Color.MAGENTA);
        p.setStyle(Paint.Style.FILL);
        Paint g = new Paint(p);
        g.setColor(Color.GREEN);

        android.graphics.Rect regionRect = makeGraphicsRect(new Rect(region_pointA, region_pointB), scaleBmpPxToCanvasPx);

        Artifact local = artifact;
        if (local != null) {
        switch (local) {
            case GREEN:
                canvas.drawRect(regionRect, g);
                break;
            case PURPLE:
                canvas.drawRect(regionRect, p);
                break;
        }
        }
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    public final Scalar BLUE = new Scalar(0, 0, 255);
    public final Scalar PURPLE = new Scalar(255.0, 0, 255);
    public final Scalar GREEN = new Scalar(0, 255, 0);

    /*
     * The core values which define the location and size of the sample regions
     */
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(65,10);
    static final int REGION_WIDTH = 40;
    static final int REGION_HEIGHT = 100;

    /*
     * Points which actually define the sample region rectangles, derived from above values
     *
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
     *
     */

    Point region_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    Mat region_H, region_S, region_V;
    Mat HSV = new Mat();
    Mat H = new Mat();
    Mat S = new Mat();
    Mat V = new Mat();
    public volatile int avgH, avgS, avgV;

    // Volatile since accessed by OpMode thread w/o synchronization
    @Nullable
    private volatile Artifact artifact = null;

    @NonNull
    private final Telemetry telemetry;

    public ColorSensorPipeline (@NonNull Telemetry telemetry) {
        this.telemetry = telemetry;
        firstRun = true;
    }

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */
    void splitInput(Mat input)
    {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(HSV, H, 0);
        Core.extractChannel(HSV, S, 1);
        Core.extractChannel(HSV, V, 2);
    }

    /*
     * Call this from the OpMode thread to obtain the latest analysis
     */
    public Artifact getAnalysis()
    {
        return artifact;
    }
}

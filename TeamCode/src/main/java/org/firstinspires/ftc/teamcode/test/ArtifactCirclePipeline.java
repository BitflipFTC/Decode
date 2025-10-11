package org.firstinspires.ftc.teamcode.test;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Locale;

public class ArtifactCirclePipeline extends OpenCvPipeline {
    /*
     * Our working image buffers
     */
    Mat yCrCbMat = new Mat();
    Mat crMat = new Mat();
    Mat cbMat = new Mat();
    Mat contoursOnPlainImageMat = new Mat();

    // purple artifacts
    Mat pCrMat = new Mat();
    Mat pCbMat = new Mat();
    Mat pThresholdMat = new Mat();
    Mat pMorphedThreshold = new Mat();

    // green artifacts
    Mat gCrMat = new Mat();
    Mat gCbMat = new Mat();
    Mat gThresholdMat = new Mat();
    Mat gMorphedThreshold = new Mat();

    MatOfPoint2f contour2f = new MatOfPoint2f();

    /*
     * The elements we use for noise reduction
     */
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));
    

    /*
     * Colors
     */
    static final Scalar TEAL = new Scalar(3, 148, 252);
    static final Scalar PURPLE = new Scalar(158, 52, 235);
    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar BLACK = new Scalar(255, 255, 255);
    static final Scalar BLUE = new Scalar(0, 0, 255);

    static final int CONTOUR_LINE_THICKNESS = 2;

    // list to hold all found artifacts
    private ArrayList<Artifact> foundArtifacts;

    public ArrayList<Artifact> getFoundArtifacts() {
        return foundArtifacts;
    }
    /*
     * Some stuff to handle returning our various buffers
     */
    enum Stage {
        FINAL,
        Cb,
        Cr,
        pThresh,
        pMorph,
        gThresh,
        gMorph,
        CONTOURS
    }


    Stage[] stages = Stage.values();

    // Keep track of what stage the viewport is showing
    int stageNum = 0;

    @Override
    public void onViewportTapped() {
        /*
         * Note that this method is invoked from the UI thread
         * so whatever we do here, we must do quickly.
         */

        int nextStageNum = stageNum + 1;

        if (nextStageNum >= stages.length) {
            nextStageNum = 0;
        }

        stageNum = nextStageNum;
    }

    @Override
    public Mat processFrame(Mat input) {
        foundArtifacts = new ArrayList<>();
        /*
         * Run the image processing
         */
        analyzeArtifacts(input, findContours(input));

        switch (stages[stageNum]) {
            case FINAL:
                return input;
            case Cb:
                return cbMat;
            case Cr:
                return crMat;
            case pThresh:
                return pThresholdMat;
            case pMorph:
                return pMorphedThreshold;
            case gThresh:
                return gThresholdMat;
            case gMorph:
                return gMorphedThreshold;
            case CONTOURS:
                return contoursOnPlainImageMat;
        }

        return input;
    }

    List<ArrayList<MatOfPoint>> findContours(Mat input) {
        // A list we'll be using to store the contours we find
        ArrayList<MatOfPoint> purpleContours = new ArrayList<>();
        ArrayList<MatOfPoint> greenContours = new ArrayList<>();

        // Convert the input image to YCrCb color space, then extract the Cb and Cr channels
        Imgproc.cvtColor(input, yCrCbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(yCrCbMat, crMat, 1);
        Core.extractChannel(yCrCbMat, cbMat, 2);

        // ------ get purple artifacts --------

        // get thresholds for cB and cR channels
        Core.inRange(crMat, new Scalar(135), new Scalar(200), pCrMat);
        Core.inRange(cbMat, new Scalar(135), new Scalar(175), pCbMat);

        // combine thresholds - if the pixel is in Cr AND Cb thresholds
        Core.bitwise_and(pCrMat, pCbMat, pThresholdMat);

        // erode and dilate
        morphMask(pThresholdMat, pMorphedThreshold);

        // Ok, now actually look for the contours! We only look for external contours.
        Imgproc.findContours(pMorphedThreshold, purpleContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);


        // ------- green artifacts ---------

        // get thresholds for cB and cR channels
        Core.inRange(crMat, new Scalar(50), new Scalar(105), gCrMat);
        Core.inRange(cbMat, new Scalar(118), new Scalar(145), gCbMat);

        // combine thresholds - if the pixel is in Cr AND Cb thresholds
        Core.bitwise_and(gCrMat, gCbMat, gThresholdMat);

        // erode and dilate
        morphMask(gThresholdMat, gMorphedThreshold);
        // Ok, now actually look for the contours! We only look for external contours.
        Imgproc.findContours(gMorphedThreshold, greenContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        greenContours.removeIf(contour -> Imgproc.contourArea(contour) <= 675);
        purpleContours.removeIf(contour -> Imgproc.contourArea(contour) <= 675);

        // We do draw the contours we find, but not to the main input buffer.
        if (stages[stageNum] == Stage.CONTOURS) {
            input.copyTo(contoursOnPlainImageMat);
            Imgproc.drawContours(contoursOnPlainImageMat, purpleContours, -1, PURPLE, CONTOUR_LINE_THICKNESS, 8);
            Imgproc.drawContours(contoursOnPlainImageMat, greenContours, -1, GREEN, CONTOUR_LINE_THICKNESS, 8);
        }

        return Arrays.asList(purpleContours, greenContours);
    }

    void morphMask(Mat input, Mat output) {
        /*
         * Apply some erosion and dilation for noise reduction.
         * A strong erosion helps separate blobs that are close together.
         */
        Imgproc.erode(input, output, erodeElement);
        Imgproc.erode(output, output, erodeElement);
        Imgproc.erode(output, output, erodeElement); // Add a third erosion

        Imgproc.dilate(output, output, dilateElement);
        Imgproc.dilate(output, output, dilateElement);
        Imgproc.dilate(output, output, dilateElement); // Add a third dilation to compensate
    }

    void analyzeArtifacts(Mat input, List<ArrayList<MatOfPoint>> contours) {
        ArrayList<MatOfPoint> purpleContours = contours.get(0);
        ArrayList<MatOfPoint> greenContours  = contours.get(1);

        for (int i = 0; i < purpleContours.size(); i++) {
            MatOfPoint contour = purpleContours.get(i);
            analyzeArtifactContour(input, contour, PURPLE, Artifact.Color.PURPLE);
        }

        for (int i = 0; i < greenContours.size(); i++) {
            MatOfPoint contour = greenContours.get(i);
            analyzeArtifactContour(input, contour, GREEN, Artifact.Color.GREEN);
        }
    }
    
    void analyzeArtifactContour(Mat input, MatOfPoint contour, Scalar drawColor, Artifact.Color color) {
        contour.convertTo(contour2f, CvType.CV_32F);

        // Fit minimum enclosing circle
        Point center = new Point();
        float[] radius = new float[1];
        Imgproc.minEnclosingCircle(contour2f, center, radius);

        // Draw filled circle on mask
        // center point
        Imgproc.circle(input, center, 2, BLACK, 2);
        Imgproc.putText(input, String.format(Locale.ENGLISH, "%s: %.2f,%.2f", color.name(), center.x, center.y), center, Imgproc.FONT_HERSHEY_PLAIN, 1, drawColor);
        // border
        Imgproc.circle(input, center, (int) radius[0], drawColor, 3);  // -1 = filled
        
        foundArtifacts.add(new Artifact(color, center));
    }

    public static class Artifact {
        Color color;
        Point center;

        public enum Color {
            PURPLE,
            GREEN
        }

        public Artifact(Color color, Point center) {
            this.color = color;
            this.center = center;
        }
    }
}

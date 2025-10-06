package org.firstinspires.ftc.teamcode.test;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ArtifactCirclePipeline extends OpenCvPipeline {
    /*
     * Our working image buffers
     */
    Mat yCrCbMat = new Mat();
    Mat crMat = new Mat();
    Mat cbMat = new Mat();


    Mat thresholdMat = new Mat();
    Mat morphedThreshold = new Mat();
    Mat contoursOnPlainImageMat = new Mat();

    /*
     * Threshold values
     */
    static final int CB_CHAN_MASK_THRESHOLD = 80;
    static final double DENSITY_UPRIGHT_THRESHOLD = 0.03;

    /*
     * The elements we use for noise reduction
     */
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(8, 8));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(11, 11));

    /*
     * Colors
     */
    static final Scalar TEAL = new Scalar(3, 148, 252);
    static final Scalar PURPLE = new Scalar(158, 52, 235);
    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar BLUE = new Scalar(0, 0, 255);

    static final int CONTOUR_LINE_THICKNESS = 2;
    static final int CB_CHAN_IDX = 2;

    static class AnalyzedStone
    {
        StoneOrientation orientation;
        double angle;
    }

    enum StoneOrientation
    {
        UPRIGHT,
        NOT_UPRIGHT
    }

    ArrayList<AnalyzedStone> internalStoneList = new ArrayList<>();
    volatile ArrayList<AnalyzedStone> clientStoneList = new ArrayList<>();

    /*
     * Some stuff to handle returning our various buffers
     */
    enum Stage
    {
        FINAL,
        Cb,
        Cr,
        CbThresh,
        CrThresh,
        MASK,
        MASK_NR,
        CONTOURS;
    }

    Stage[] stages = Stage.values();

    // Keep track of what stage the viewport is showing
    int stageNum = 0;

    @Override
    public void onViewportTapped()
    {
        /*
         * Note that this method is invoked from the UI thread
         * so whatever we do here, we must do quickly.
         */

        int nextStageNum = stageNum + 1;

        if(nextStageNum >= stages.length)
        {
            nextStageNum = 0;
        }

        stageNum = nextStageNum;
    }

    @Override
    public Mat processFrame(Mat input)
    {
        // We'll be updating this with new data below
        internalStoneList.clear();

        /*
         * Run the image processing
         */
        analyzeArtifacts(input, findContours(input));

        clientStoneList = new ArrayList<>(internalStoneList);

        switch (stages[stageNum])
        {
            case Cb:
            {
                return cbMat;
            }

            case Cr:
            {
                return crMat;
            }

            case FINAL:
            {
                return input;
            }

            case MASK:
            {
                return thresholdMat;
            }

            case MASK_NR:
            {
                return morphedThreshold;
            }

            case CONTOURS:
            {
                return contoursOnPlainImageMat;
            }
        }

        return input;
    }

    public ArrayList<AnalyzedStone> getDetectedStones()
    {
        return clientStoneList;
    }

    Artifacts findContours(Mat input)
    {
        /*
            public static final ColorRange ARTIFACT_GREEN = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar( 32,  50, 118),
            new Scalar(255, 105, 145)
    );

    public static final ColorRange ARTIFACT_PURPLE = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar( 32, 135, 135),
            new Scalar(255, 155, 169)
    );
    */

        // A list we'll be using to store the contours we find
        Artifacts artifacts = new Artifacts();
        Imgproc.cvtColor(input, yCrCbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(yCrCbMat, crMat, 1);
        Core.extractChannel(yCrCbMat, cbMat, 2);

        // ------ get purple artifacts --------

        // Convert the input image to YCrCb color space, then extract the Cb and Cr channels
        Mat pCrMat = new Mat();
        Mat pCbMat = new Mat();
        Mat pThresholdMat = new Mat();
        Mat pMorphedThreshold = new Mat();

        // get thresholds for cB and cR channels
        Core.inRange(crMat, new Scalar(135),new Scalar(200),pCrMat);
        Core.inRange(cbMat,new Scalar(135), new Scalar(175), pCbMat);

        // combine thresholds - if the pixel is in Cr AND Cb thresholds
        Core.bitwise_and(pCrMat,pCbMat,pThresholdMat);

        // erode and dilate
        morphMask(pThresholdMat, pMorphedThreshold);

        // Ok, now actually look for the contours! We only look for external contours.
        Imgproc.findContours(pMorphedThreshold, artifacts.purple, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        artifacts.purple.removeIf(contour -> Imgproc.contourArea(contour) <= 2500);


        // ------- green artifacts ---------
        Mat gCrMat = new Mat();
        Mat gCbMat = new Mat();
        Mat gThresholdMat = new Mat();
        Mat gMorphedThreshold = new Mat();

        // Convert the input image to YCrCb color space, then extract the Cb and Cr channels

        // get thresholds for cB and cR channels

        //            new Scalar( 32,  50, 118),
        //            new Scalar(255, 105, 145)
        Core.inRange(crMat, new Scalar(50),new Scalar(105),gCrMat);
        Core.inRange(cbMat,new Scalar(118), new Scalar(145), gCbMat);

        // combine thresholds - if the pixel is in Cr AND Cb thresholds
        Core.bitwise_and(gCrMat,gCbMat,gThresholdMat);

        // erode and dilate
        morphMask(gThresholdMat, gMorphedThreshold);
        // Ok, now actually look for the contours! We only look for external contours.
        Imgproc.findContours(gMorphedThreshold, artifacts.green, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        // We do draw the contours we find, but not to the main input buffer.
        artifacts.green.removeIf(contour -> Imgproc.contourArea(contour) <= 2500);

        input.copyTo(contoursOnPlainImageMat);
        Imgproc.drawContours(contoursOnPlainImageMat, artifacts.purple, -1, PURPLE, CONTOUR_LINE_THICKNESS, 8);
        Imgproc.drawContours(contoursOnPlainImageMat, artifacts.green, -1, GREEN, CONTOUR_LINE_THICKNESS, 8);

        return artifacts;
    }

    void morphMask(Mat input, Mat output)
    {
        /*
         * Apply some erosion and dilation for noise reduction
         */

        Imgproc.erode(input, output, erodeElement);
        Imgproc.erode(output, output, erodeElement);

        Imgproc.dilate(output, output, dilateElement);
        Imgproc.dilate(output, output, dilateElement);
    }

    void analyzeArtifacts(Mat input, Artifacts artifacts)
    {
        // Transform the contour to a different format
//        Point[] points = contour.toArray();
//        MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

        for (MatOfPoint contour : artifacts.purple) {
            MatOfPoint2f contour2f = new MatOfPoint2f();
            contour.convertTo(contour2f, CvType.CV_32F);

            // Fit minimum enclosing circle
            Point center = new Point();
            float[] radius = new float[1];
            Imgproc.minEnclosingCircle(contour2f, center, radius);

            // Draw filled circle on mask
            Imgproc.circle(input, center,2, new Scalar(255,255,255), 2);
            Imgproc.putText(input,"PURPLE:" + radius[0],center,Imgproc.FONT_HERSHEY_PLAIN,1,new Scalar(255,255,255));
            Imgproc.circle(input, center, (int) radius[0], new Scalar(255,0,255), 3);  // -1 = filled
        }

        for (MatOfPoint contour : artifacts.green) {
            MatOfPoint2f contour2f = new MatOfPoint2f();
            contour.convertTo(contour2f, CvType.CV_32F);

            // Fit minimum enclosing circle
            Point center = new Point();
            float[] radius = new float[1];
            Imgproc.minEnclosingCircle(contour2f, center, radius);

            // Draw filled circle on mask
            Imgproc.circle(input, center,2, new Scalar(255,255,255), 2);
            Imgproc.putText(input,"GREEN:" + radius[0],center,Imgproc.FONT_HERSHEY_PLAIN,1,new Scalar(255,255,255));
            Imgproc.circle(input, center, (int) radius[0], new Scalar(0,255,0), 3);  // -1 = filled
        }


        // Do a rect fit to the contour, and draw it on the screen
//        RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);
//        drawRotatedRect(rotatedRectFitToContour, input);
    }

    static class Artifacts {
        ArrayList<MatOfPoint> purple;
        ArrayList<MatOfPoint> green;

        Artifacts() {
            purple = new ArrayList<>();
            green = new ArrayList<>();
        }
    }

    static class ContourRegionAnalysis
    {
        /*
         * This class holds the results of analyzeContourRegion()
         */

        double hullArea;
        double contourArea;
        double density;
        List<MatOfPoint> listHolderOfMatOfPoint;
    }

    static ContourRegionAnalysis analyzeContourRegion(ArrayList<Point> contourPoints)
    {
        // drawContours() requires a LIST of contours (there's no singular drawContour()
        // method), so we have to make a list, even though we're only going to use a single
        // position in it...
        MatOfPoint matOfPoint = new MatOfPoint();
        matOfPoint.fromList(contourPoints);
        List<MatOfPoint> listHolderOfMatOfPoint = List.of(matOfPoint);

        // Compute the convex hull of the contour
        MatOfInt hullMatOfInt = new MatOfInt();
        Imgproc.convexHull(matOfPoint, hullMatOfInt);

        // Was the convex hull calculation successful?
        if(hullMatOfInt.toArray().length > 0)
        {
            // The convex hull calculation tells us the INDEX of the points which
            // which were passed in eariler which form the convex hull. That's all
            // well and good, but now we need filter out that original list to find
            // the actual POINTS which form the convex hull
            Point[] hullPoints = new Point[hullMatOfInt.rows()];
            List<Integer> hullContourIdxList = hullMatOfInt.toList();

            for (int i = 0; i < hullContourIdxList.size(); i++)
            {
                hullPoints[i] = contourPoints.get(hullContourIdxList.get(i));
            }

            ContourRegionAnalysis analysis = new ContourRegionAnalysis();
            analysis.listHolderOfMatOfPoint = listHolderOfMatOfPoint;

            // Compute the hull area
            analysis.hullArea = Imgproc.contourArea(new MatOfPoint(hullPoints));

            // Compute the original contour area
            analysis.contourArea = Imgproc.contourArea(listHolderOfMatOfPoint.get(0));

            // Compute the contour density. This is the ratio of the contour area to the
            // area of the convex hull formed by the contour
            analysis.density = analysis.contourArea / analysis.hullArea;

            return analysis;
        }
        else
        {
            return null;
        }
    }

    static void drawTagText(RotatedRect rect, String text, Mat mat)
    {
        Imgproc.putText(
                mat, // The buffer we're drawing on
                text, // The text we're drawing
                new Point( // The anchor point for the text
                        rect.center.x-50,  // x anchor point
                        rect.center.y+25), // y anchor point
                Imgproc.FONT_HERSHEY_PLAIN, // Font
                1, // Font size
                TEAL, // Font color
                1); // Font thickness
    }

    static void drawRotatedRect(RotatedRect rect, Mat drawOn)
    {
        /*
         * Draws a rotated rect by drawing each of the 4 lines individually
         */

        Point[] points = new Point[4];
        rect.points(points);

        for(int i = 0; i < 4; ++i)
        {
            Imgproc.line(drawOn, points[i], points[(i+1)%4], RED, 2);
        }
    }
}

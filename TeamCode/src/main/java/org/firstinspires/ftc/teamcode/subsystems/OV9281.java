package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import androidx.annotation.Nullable;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.MotifPattern;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

@Configurable
public class OV9281 implements Subsystem {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private ExposureControl exposureControl;
    private GainControl gainControl;
    private long defaultExposure;
    private int defaultGain;
    boolean debugTelemetry = true;
    public int viewContainerId = -1;

    private final ArrayList<AprilTagDetection> detectionsBuffer = new ArrayList<>();
    private final ArrayList<AprilTagDetection> obeliskDetections = new ArrayList<>();

    // https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_localization/apriltag-localization.html
    /*
        If all values are zero (no translation), that implies the camera is at the center of the robot.
        Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12 inches above the ground - you would need to set the position to (-5, 7, 12).
     */
    private final Position cameraPosition = new Position(DistanceUnit.MM,
            0.03380, 212.51288, 271.44038, 0);

    private Pose robotPose = new Pose();
    private MedianPoseFilter filter = new MedianPoseFilter();
    private Pose newPose = new Pose();
    private Pose2D robotPose2d;
    private boolean newReading = false;

    /*
        If all values are zero (no rotation), that implies the camera is pointing straight up.
        In most cases, you’ll need to set the pitch to -90 degrees (rotation about the x-axis), meaning the camera is horizontal.
        Use a yaw of 0 if the camera is pointing forwards, +90 degrees if it’s pointing straight left, -90 degrees for straight right, etc.
        You can also set the roll to +/-90 degrees if it’s vertical, or 180 degrees if it’s upside-down.
     */
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -75, 0, 0);

    double fx = 549.993552641,
            fy = 549.993552641,
            cx = 327.021677114,
            cy = 255.879397051;

    public AprilTagProcessor getAprilTag() {
        return aprilTag;
    }

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    // exposure: 1-7
    // gain: 1-6
    @Override
    public void initialize () {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(fx,fy,cx,cy)
                .setNumThreads(3)
                .setCameraPose(cameraPosition,cameraOrientation)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        aprilTag.setDecimation(1f);

        if (viewContainerId == -1) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(ActiveOpMode.hardwareMap().get(WebcamName.class, "camera"))
                    .setCameraResolution(new Size(640, 480))
                    .setShowStatsOverlay(true)
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .enableLiveView(true)
                    .addProcessor(aprilTag)
                    .setAutoStopLiveView(true)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(ActiveOpMode.hardwareMap().get(WebcamName.class, "camera"))
                    .setCameraResolution(new Size(640, 480))
                    .setShowStatsOverlay(true)
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .setLiveViewContainerId(viewContainerId)
                    .addProcessor(aprilTag)
                    .setAutoStopLiveView(true)
                    .build();
        }

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }

        exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        gainControl = visionPortal.getCameraControl(GainControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        defaultExposure = exposureControl.getExposure(TimeUnit.MILLISECONDS);
        defaultGain = gainControl.getGain();
        exposureControl.setExposure(6, TimeUnit.MILLISECONDS);
        gainControl.setGain(7);
    }

    public void resetExposureGain () {
        if (exposureControl != null && gainControl != null) {
            exposureControl.setMode(ExposureControl.Mode.Auto);
            exposureControl.setExposure(defaultExposure,TimeUnit.MILLISECONDS);
            gainControl.setGain(defaultGain);
        }
    }

    public void setExposure (int exposure) {
        exposureControl.setExposure(exposure, TimeUnit.MILLISECONDS);
    }

    public void disableProcessor() {
        visionPortal.setProcessorEnabled(aprilTag, false);
    }

    public void enableProcessor() {
        visionPortal.setProcessorEnabled(aprilTag, true);
    }

    public void stopStreaming() {
        visionPortal.stopStreaming();
    }

    public void resumeStreaming() {
        visionPortal.resumeStreaming();
    }

    public MotifPattern getMotif() {
        obeliskDetections.clear();

        for (AprilTagDetection d : this.aprilTag.getDetections()) {
            if (d.metadata != null && d.id >= 21 && d.id <= 23) {
                obeliskDetections.add(d);
            }
        }

        @Nullable
        MotifPattern pattern = null;

        if (obeliskDetections.isEmpty()) return pattern;

        for (AprilTagDetection detection : obeliskDetections) {
            switch (detection.metadata.name) {
                case "Obelisk_GPP": pattern = MotifPattern.GPP; break;
                case "Obelisk_PGP": pattern = MotifPattern.PGP; break;
                case "Obelisk_PPG": pattern = MotifPattern.PPG; break;
            }
        }

        return pattern;
    }

    @Override
    public void periodic() {
        detectionsBuffer.clear();
        if (aprilTag.getDetections() != null) {
            detectionsBuffer.addAll(aprilTag.getDetections());
        }

        int count = detectionsBuffer.size();

        if (count == 0) {
            if (debugTelemetry)
                ActiveOpMode.telemetry().addData("Detected April Tags", 0);
            newReading = false;
            return;
        }

        if (debugTelemetry)
            ActiveOpMode.telemetry().addData("Detected April Tags", detectionsBuffer.size());
        for (AprilTagDetection detection : detectionsBuffer) {
            if (detection.metadata == null) {
//                if (debugTelemetry)
//                    ActiveOpMode.telemetry().addData("Current tag", "No metadata");
                newReading = false;
                continue;
            }


            if (debugTelemetry) {
                ActiveOpMode.telemetry().addData("TAG NAME", detection.metadata.name);
//                ActiveOpMode.telemetry().addData("ID", detection.id);
                ActiveOpMode.telemetry().addData("Sureness", detection.decisionMargin);
            }

            if ((detection.id == 20 || detection.id == 24) && detection.decisionMargin > 26.7) {
                robotPose2d = new Pose2D(
                        DistanceUnit.INCH, detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, AngleUnit.DEGREES, detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)
                );
                robotPose = filter.update(new Pose(robotPose2d.getY(DistanceUnit.INCH) + 72.0, (-1 * robotPose2d.getX(DistanceUnit.INCH)) + 72.0, robotPose2d.getHeading(AngleUnit.RADIANS)));

                newReading = true;
            } else {
                newReading = false;
            }
        }
    }

    public void setDecimation(float decimation) {
        aprilTag.setDecimation(decimation);
    }

    public int getDetectionsAmount() {
        return detectionsBuffer.size();
    }

    public Pose getRobotPose() {
        return robotPose;
    }

    public Pose getNewPose() {
        return newPose;
    }

    public Pose2D getRobotPose2d () {
        return robotPose2d;
    }

    public boolean getHasNewReading() {
        return newReading;
    }
}

class MedianPoseFilter {

    private final Deque<Pose> buffer = new ArrayDeque<>(3);

    public Pose update(Pose newPose) {
        if (buffer.size() == 3) {
            buffer.removeFirst();
        }
        buffer.addLast(newPose);

        if (buffer.size() < 3) {
            return newPose;
        }

        Pose[] p = buffer.toArray(new Pose[0]);

        double x = median3(p[0].getX(), p[1].getX(), p[2].getX());
        double y = median3(p[0].getY(), p[1].getY(), p[2].getY());
        double heading = medianAngle3(
                p[0].getHeading(),
                p[1].getHeading(),
                p[2].getHeading()
        );

        return new Pose(x, y, heading, newPose.getCoordinateSystem());
    }

    private double median3(double a, double b, double c) {
        if (a > b) {
            if (b > c) return b;
            else return Math.min(a, c);
        } else {
            if (a > c) return a;
            else return Math.min(b, c);
        }
    }

    private double medianAngle3(double a, double b, double c) {
        double mx = median3(Math.cos(a), Math.cos(b), Math.cos(c));
        double my = median3(Math.sin(a), Math.sin(b), Math.sin(c));
        return Math.atan2(my, mx);
    }
}

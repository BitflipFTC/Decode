package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import androidx.annotation.Nullable;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.MotifPattern;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
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

    private final ArrayList<AprilTagDetection> detectionsBuffer = new ArrayList<>();

    // https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_localization/apriltag-localization.html
    /*
        If all values are zero (no translation), that implies the camera is at the center of the robot.
        Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12 inches above the ground - you would need to set the position to (-5, 7, 12).
     */
    private final Position cameraPosition = new Position(DistanceUnit.MM,
            0.03380, 212.51288, 271.44038, 0);

    private Pose robotPose = new Pose();

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

    private final double lowPass = 0.075;

    // exposure: 1-7
    // gain: 1-6
    @Override
    public void initialize () {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
//                .setDrawTagOutline(true)
                .setDrawTagID(true)
//                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(fx,fy,cx,cy)
                .setCameraPose(cameraPosition,cameraOrientation)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        aprilTag.setDecimation(3f);
        aprilTag.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_SOLVEPNP_EPNP);

        visionPortal = new VisionPortal.Builder()
                .setCamera(ActiveOpMode.hardwareMap().get(WebcamName.class, "camera"))
                .setCameraResolution(new Size(640,480))
                .setShowStatsOverlay(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .addProcessor(aprilTag)
                .setAutoStopLiveView(true)
                .build();

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
        exposureControl.setExposure(4, TimeUnit.MILLISECONDS);
        gainControl.setGain(6);
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
        ArrayList<AprilTagDetection> obeliskDetections = this.aprilTag.getDetections().stream().filter((detection) -> detection.metadata.name.contains("Obelisk")).collect(Collectors.toCollection(ArrayList::new));
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
            return;
        }

        if (debugTelemetry)
            ActiveOpMode.telemetry().addData("Detected April Tags", detectionsBuffer.size());
        for (AprilTagDetection detection : detectionsBuffer) {
            if (detection.metadata == null) {
                if (debugTelemetry)
                    ActiveOpMode.telemetry().addData("Current tag", "No metadata");
                continue;
            }

            if (debugTelemetry)
                ActiveOpMode.telemetry().addData("TAG NAME", detection.metadata.name);

            if ((detection.id == 20 || detection.id == 24) && detection.decisionMargin > 30f) {
                robotPose = PoseConverter.pose2DToPose(new Pose2D(
                        DistanceUnit.INCH, detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, AngleUnit.DEGREES, detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)
                ), InvertedFTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
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
}

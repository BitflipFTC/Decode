package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.OpModeConstants.hardwareMap;
import static org.firstinspires.ftc.teamcode.util.OpModeConstants.telemetry;

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
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.MotifPattern;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;

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

    private double currentTagBearing = 0;
    private double lastDistanceToGoal = 0;
    private double distanceToGoal = 0;
    private int targetID = 0;
    // 20: Blue
    // 24: Red

    // exposure: 1-7
    // gain: 1-6
    public OV9281 () {
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

        assert hardwareMap != null;
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
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
        ArrayList<AprilTagDetection> currentDetections = this.aprilTag.getDetections();
        ArrayList<AprilTagDetection> obeliskDetections = currentDetections.stream().filter((detection) -> detection.metadata.name.contains("Obelisk")).collect(Collectors.toCollection(ArrayList::new));
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
        lastDistanceToGoal = distanceToGoal < 0 ? lastDistanceToGoal : distanceToGoal;
        detectionsBuffer.clear();
        if (aprilTag.getDetections() != null) {
            detectionsBuffer.addAll(aprilTag.getDetections());
        }

        int count = detectionsBuffer.size();

        assert telemetry != null;
        if (count == 0) {
            if (debugTelemetry) {
                telemetry.addData("Detected April Tags", 0);
            }
            distanceToGoal = -1.0;
            currentTagBearing = 0.0;
            return;
        }

        if (debugTelemetry)
            telemetry.addData("Detected April Tags", detectionsBuffer.size());
        for (AprilTagDetection detection : detectionsBuffer) {
            if (detection.metadata == null) {
                distanceToGoal = -1.0;
                currentTagBearing = -0.1;
                if (debugTelemetry)
                    telemetry.addData("Current tag", "No metadata");
                continue;
            }

            if (debugTelemetry)
                telemetry.addData("TAG NAME", detection.metadata.name);

            if (detection.id == targetID) {
                distanceToGoal = detection.ftcPose.range * lowPass + (1 - lowPass) * lastDistanceToGoal;
                currentTagBearing = detection.ftcPose.bearing;
                robotPose = PoseConverter.pose2DToPose(new Pose2D(
                        DistanceUnit.INCH, detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, AngleUnit.DEGREES, detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)
                ), InvertedFTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
            } else {
                distanceToGoal = -1.0;
                currentTagBearing = -0.2;
            }
        }

        if (debugTelemetry) {
            telemetry.addData("Target Tag ID", targetID);
            telemetry.addData("Distance from goal", "%06.3fin", distanceToGoal);
            telemetry.addData("Current tag bearing", "%05.2f deg", currentTagBearing);
            telemetry.addLine("---------------------------");
        }
    }

    public void setTargetID (int id) {
        targetID = id;
    }

    public int getTargetID() {
        return targetID;
    }

    public void setDecimation(float decimation) {
        aprilTag.setDecimation(decimation);
    }

    /**
     * @return the bearing to the detected apriltag in degrees.
     */
    public double getCurrentTagBearing() {
        return currentTagBearing;
    }

    public void resetAprilTagData() {
        currentTagBearing = 0.0;
        distanceToGoal = 0.0;
    }

    /**
     * @return the distance to the detected apriltag in inches.
     */
    public double getDistanceToGoal() {
        return distanceToGoal;
    }

    public int getDetectionsAmount() {
        return detectionsBuffer.size();
    }

    public Pose getRobotPose() {
        return robotPose;
    }
}

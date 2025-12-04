package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

    private final ArrayList<AprilTagDetection> detectionsBuffer = new ArrayList<>();

    // these are relative to the center of rotation of the turret.
    private final Position cameraPosition = new Position(DistanceUnit.MM,
            -0.02, -41.29940, 0, 0);

    // with all orientation values 0, the camera is facing straight up. since we want it
    // to be facing forward, we do -90.
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

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
    @Override
    public void initialize () {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(fx,fy,cx,cy)
                .setCameraPose(cameraPosition,cameraOrientation)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        aprilTag.setDecimation(2f);
        aprilTag.setPoseSolver(AprilTagProcessor.PoseSolver.APRILTAG_BUILTIN);

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

    public void stopStreaming() {
        visionPortal.stopStreaming();
    }

    public void resumeStreaming() {
        visionPortal.resumeStreaming();
    }

    public MotifPattern getMotif() {
        ArrayList<AprilTagDetection> currentDetections = this.aprilTag.getDetections();
        ArrayList<AprilTagDetection> obeliskDetections = currentDetections.stream().filter((detection) -> detection.metadata.name.contains("Obelisk")).collect(Collectors.toCollection(ArrayList::new));
        MotifPattern pattern = MotifPattern.NONE;

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

        if (count == 0) {
            ActiveOpMode.telemetry().addData("Detected April Tags", 0);
            distanceToGoal = -1.0;
            currentTagBearing = 0.0;
            return;
        }

        ActiveOpMode.telemetry().addData("Detected April Tags", detectionsBuffer.size());
        for (AprilTagDetection detection : detectionsBuffer) {
            if (detection.metadata == null) {
                distanceToGoal = -1.0;
                currentTagBearing = -0.1;
                ActiveOpMode.telemetry().addData("Current tag", "No metadata");
                continue;
            }

            ActiveOpMode.telemetry().addData("TAG NAME", detection.metadata.name);

            if (detection.id == targetID) {
                distanceToGoal = detection.ftcPose.range * lowPass + (1 - lowPass) * lastDistanceToGoal;
                currentTagBearing = detection.ftcPose.bearing;
            } else {
                distanceToGoal = -1.0;
                currentTagBearing = -0.2;
            }
        }
        ActiveOpMode.telemetry().addData("Target Tag ID", targetID);
        ActiveOpMode.telemetry().addData("Distance from goal", "%06.3fin", distanceToGoal);
        ActiveOpMode.telemetry().addData("Current tag bearing", "%05.2f deg",currentTagBearing);
        ActiveOpMode.telemetry().addLine("---------------------------");
    }

    public void setTargetID (int id) {
        targetID = id;
    }

    public int getTargetID() {
        return targetID;
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
}

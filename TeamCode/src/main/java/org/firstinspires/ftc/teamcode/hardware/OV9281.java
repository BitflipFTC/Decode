package org.firstinspires.ftc.teamcode.hardware;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.MotifPattern;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;

import dev.frozenmilk.util.modifier.BiModifier;


public class OV9281 extends SubsystemBase {

    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;
    private ExposureControl exposureControl;
    private GainControl gainControl;
    private long defaultExposure;
    private int defaultGain;

    private final Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
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

    private double currentTagBearing = 0;
    private double distanceToGoal = 0;
    private int targetID = 0;
    // 20: Blue
    // 24: Red

    private Telemetry telemetry = null;

    // exposure: 1-7
    // gain: 1-6
    public OV9281 (OpMode opMode, int exposureMS, int gain) {
        telemetry = opMode.telemetry;

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

        aprilTag.setDecimation(3f);
        aprilTag.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_SOLVEPNP_EPNP);

        visionPortal = new VisionPortal.Builder()
                .setCamera(opMode.hardwareMap.get(WebcamName.class, "camera"))
                .setCameraResolution(new Size(640,480))
                .setShowStatsOverlay(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(false)
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

        if (exposureMS != 0 && gain != 0) {
            exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            gainControl = visionPortal.getCameraControl(GainControl.class);

            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }

            exposureControl.setMode(ExposureControl.Mode.Manual);

            defaultExposure = exposureControl.getExposure(TimeUnit.MILLISECONDS);
            defaultGain = gainControl.getGain();

            exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
            gainControl.setGain(gain);
        }
    }

    public OV9281(OpMode opMode) {
        this(opMode,0,0);
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

        if (!obeliskDetections.isEmpty()) {
            for (AprilTagDetection detection : obeliskDetections) {
                switch (detection.metadata.name) {
                    case "Obelisk_GPP":
                        pattern = MotifPattern.GPP;
                        break;
                    case "Obelisk_PGP":
                        pattern = MotifPattern.PGP;
                        break;
                    case "Obelisk_PPG":
                        pattern = MotifPattern.PPG;
                        break;
                }
            }
        }

        return pattern;
    }

    @Override
    public void periodic() {
        ArrayList<AprilTagDetection> currentDetections = aprilTag.getDetections();

        if (!currentDetections.isEmpty()) {
            telemetry.addData("Detected April Tags", currentDetections.size());

            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    telemetry.addData("TAG NAME", detection.metadata.name);

                    if (detection.id == targetID) {
                        distanceToGoal = detection.ftcPose.range;
                        currentTagBearing = detection.ftcPose.bearing;
                    } else {
                        distanceToGoal = -1.0;
                        currentTagBearing = -1.0;
                    }

                } else {
                    distanceToGoal = -1.0;
                    currentTagBearing = -1.0;
                    telemetry.addData("Current tag", "No metadata");
                }
            }
        } else {
            telemetry.addData("Detected April Tags", 0);
            distanceToGoal = -1.0;
            currentTagBearing = -1.0;
        }

        telemetry.addLine("---------------------------");
    }

    public void setTargetID (int id) {
        targetID = id;
    }

    public int getTargetID() {
        return targetID;
    }

    public double getCurrentTagBearing() {
        return currentTagBearing;
    }

    public double getDistanceToGoal() {
        return distanceToGoal;
    }
}

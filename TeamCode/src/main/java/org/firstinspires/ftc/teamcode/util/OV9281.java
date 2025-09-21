package org.firstinspires.ftc.teamcode.util;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;


public class OV9281 {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private ExposureControl exposureControl;
    private GainControl gainControl;
    private long defaultExposure;
    private int defaultGain;

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
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

    // exposure: 1-7
    // gain: 1-6
    public OV9281 (OpMode opMode, int exposureMS, int gain) {
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

        visionPortal = new VisionPortal.Builder()
                .setCamera(opMode.hardwareMap.get(WebcamName.class, "camera"))
                .setCameraResolution(new Size(640,480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .addProcessor(aprilTag)
                .build();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }

        if (exposureMS != 0 && gain != 01) {
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
}

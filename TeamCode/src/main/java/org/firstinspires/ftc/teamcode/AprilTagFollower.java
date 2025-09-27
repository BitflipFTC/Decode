/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.utils.LoopTimer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.AprilTagTrackerPID;
import org.firstinspires.ftc.teamcode.util.OV9281;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.SquidController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Configurable
@TeleOp(name = "Concept: AprilTag Tracker", group = "Concept")
public class AprilTagFollower extends LinearOpMode {
    private OV9281 camera;
    LoopTimer loopTimer;
    DcMotorEx front_left;
    DcMotorEx front_right;
    DcMotorEx back_left;
    DcMotorEx back_right;
    double driveSpeed = 1;
    public static double targetTagPos = 320;
    public static double currentTagPos;

    private PIDController controller;

    @Override
    public void runOpMode() {

        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry, FtcDashboard.getInstance().getTelemetry());
        camera = new OV9281(this,3,6);

        controller = new SquidController();
        loopTimer = new LoopTimer();

        front_left  = hardwareMap.get(DcMotorEx.class, "frontleft");
        front_right = hardwareMap.get(DcMotorEx.class, "frontright");
        back_left   = hardwareMap.get(DcMotorEx.class, "backleft");
        back_right  = hardwareMap.get(DcMotorEx.class, "backright");

        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Initialized", true);
        telemetry.update();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        loopTimer.start();


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        while (opModeIsActive()) {

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            telemetryAprilTag();

            // Push telemetry to the Driver Station.

            // Save CPU resources; can resume streaming when needed.
            if (gamepad1.dpad_down) {
                camera.getVisionPortal().stopStreaming();
            } else if (gamepad1.dpad_up) {
                camera.getVisionPortal().resumeStreaming();
            }

            telemetry.addData("Current speed value", driveSpeed);
            telemetry.addData("Current tag pos", currentTagPos);

            controller.setCoeffs(AprilTagTrackerPID.p, AprilTagTrackerPID.i, AprilTagTrackerPID.d, 0);
            controller.setIntegrationBounds(AprilTagTrackerPID.min, AprilTagTrackerPID.max);

            double pidError = controller.calculate(currentTagPos, targetTagPos);

            telemetry.addData("pid Error", pidError);

            if (Math.abs(pidError) >= 0.115) {
                front_left.setPower(-pidError);
                front_right.setPower(pidError);
                back_left.setPower(-pidError);
                back_right.setPower(pidError);
            }

            telemetry.addData("Loop time","%dms",loopTimer.getMs());
            telemetry.addData("Target Tag Pos", targetTagPos);
            telemetry.update();
        }

        // Save more CPU resources when camera is no longer needed.
        camera.getVisionPortal().close();

    }   // end method runOpMode()

    /**
     * Add telemetry about AprilTag detections.
     */
    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = camera.getAprilTag().getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                // Only use tags that don't have Obelisk in them
                if (!detection.metadata.name.contains("Obelisk")) {
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                }
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }

            currentTagPos = detection.center.x;
        }   // end for() loop

        if(currentDetections.isEmpty()) {
            // stop turning (because there is nothing to turn to)
            currentTagPos = 320;
        }

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

}   // end class

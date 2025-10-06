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

package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.util.AprilTagAutoPID.targetTagPos;

import static java.lang.Math.abs;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.utils.LoopTimer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.AprilTagAutoPID;
import org.firstinspires.ftc.teamcode.hardware.OV9281;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.SquidController;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Configurable
@TeleOp(name = "Concept: AprilTag Auto", group = "Concept")
public class AprilTagAuto extends LinearOpMode {
    private OV9281 camera;
    LoopTimer loopTimer;
    DcMotorEx front_left;
    DcMotorEx front_right;
    DcMotorEx back_left;
    DcMotorEx back_right;
    double driveSpeed = 1;
    public static double currentTagPos;

    private PIDController controller;

    @Override
    public void runOpMode() {

        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry, FtcDashboard.getInstance().getTelemetry());
        camera = new OV9281(this,4,6);

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

            if (gamepad1.a) {
                targetTagPos = 160;
            } else if (gamepad1.b) {
                targetTagPos = 320;
            } else if (gamepad1.x) {
                targetTagPos = 480;
            }

            telemetry.addData("Current speed value", driveSpeed);
            telemetry.addData("Current tag pos", currentTagPos);
            telemetry.addData("Target Tag Pos", targetTagPos);
            telemetry.addData("Loop time","%dms",loopTimer.getMs());

            controller.setCoeffs(AprilTagAutoPID.p, AprilTagAutoPID.i, AprilTagAutoPID.d, 0);
            controller.setIntegrationBounds(AprilTagAutoPID.min, AprilTagAutoPID.max);

            double pidError = controller.calculate(currentTagPos, targetTagPos);

            telemetry.addData("pid Error", pidError);

            if (abs(pidError) > 0.05) {
                front_left.setPower(-pidError);
                front_right.setPower(pidError);
                back_left.setPower(-pidError);
                back_right.setPower(pidError);
            } else {
                front_left.setPower(0);
                front_right.setPower(0);
                back_left.setPower(0);
                back_right.setPower(0);
            }

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


        /*
        ======================================
        DECODE TAG LIBRARY
        ======================================

        public static AprilTagLibrary getDecodeTagLibrary(){
        return new AprilTagLibrary.Builder()
                .addTag(20, "BlueTarget",
                        6.5, new VectorF(-58.3727f, -55.6425f, 29.5f), DistanceUnit.INCH,
                        new Quaternion(0.2182149f, -0.2182149f, -0.6725937f, 0.6725937f, 0))
                .addTag(21, "Obelisk_GPP",
                        6.5, DistanceUnit.INCH)
                .addTag(22, "Obelisk_PGP",
                        6.5, DistanceUnit.INCH)
                .addTag(23, "Obelisk_PPG",
                        6.5, DistanceUnit.INCH)
                .addTag(24, "RedTarget",
                        6.5, new VectorF(-58.3727f, 55.6425f, 29.5f), DistanceUnit.INCH,
                        new Quaternion(0.6725937f, -0.6725937f, -0.2182149f, 0.2182149f, 0))
                .build();
         }

         Given the targets on the left, Blue on the bottom, red on the top
         */

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.metadata.name.contains("Obelisk")) {
                    telemetry.addLine("Obelisk Tag:");
                }
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                // Only use tags that don't have Obelisk in them
                if (!detection.metadata.name.contains("Obelisk")) {
                    Pose2D tagPos = new Pose2D(DistanceUnit.INCH, detection.metadata.fieldPosition.get(0), detection.metadata.fieldPosition.get(1),
                            AngleUnit.DEGREES, detection.metadata.fieldOrientation.toOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ,AngleUnit.DEGREES).secondAngle); // given north is 0, clockwise
                    Pose2D robotPos = new Pose2D(detection.robotPose.getPosition().unit, detection.robotPose.getPosition().x, detection.robotPose.getPosition().y,
                            AngleUnit.DEGREES, detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES));

                    double dist = Math.sqrt(
                            Math.pow(
                                    tagPos.getX(DistanceUnit.INCH) - robotPos.getX(DistanceUnit.INCH),
                                    2
                            )
                            + Math.pow(
                                    tagPos.getY(DistanceUnit.INCH) - robotPos.getY(DistanceUnit.INCH),
                                    2
                            )
                    );

                    telemetry.addData("Distance to tag", dist);
                    telemetry.addData("aprilTag Rot", tagPos.getHeading(AngleUnit.DEGREES));
                    telemetry.addData("Robot Rot", robotPos.getHeading(AngleUnit.DEGREES));

                    // sqrt (( pow ( tag.x - rob.x ), 2 ) + ( pow ( tag.y - rob.y ), 2 ))
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
            currentTagPos = targetTagPos;
        }

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()


}   // end class

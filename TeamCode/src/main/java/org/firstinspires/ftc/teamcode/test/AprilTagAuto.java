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

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Disabled
@TeleOp(name = "Concept: AprilTag Auto", group = "Concept")
public class AprilTagAuto extends LinearOpMode {
    private WebcamName camera;
    private VisionPortal visionPortal;
    private AprilTagProcessor processor;
    DcMotorEx front_left;
    DcMotorEx front_right;
    DcMotorEx back_left;
    DcMotorEx back_right;
    double driveSpeed = 1;

    // this is the bearing of the apriltag relative to your camera (positive means the tag is on the left half of the FOV)
    public static double currentTagBearing;
    public static double targetTagBearing = 0;

    // this is a value that you tune so that your robot turns to face the apriltag without overshooting or oscillating
    public static double kP = 0.011;

    @Override
    public void runOpMode() {
        // this is the default name for a webcam in the hardware map, change it to whatever
        // your camera is actually named
        camera = hardwareMap.get(WebcamName.class, "Webcam 1");

        // these set up stuff for camera processing
        processor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(camera, processor);

        // declare drive motors, change to whatever your actual motors' names are
        front_left  = hardwareMap.get(DcMotorEx.class, "frontleft");
        front_right = hardwareMap.get(DcMotorEx.class, "frontright");
        back_left   = hardwareMap.get(DcMotorEx.class, "backleft");
        back_right  = hardwareMap.get(DcMotorEx.class, "backright");

        // reverse whatever motors you normally reverse
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            aprilTagProcessing();

            telemetry.addData("Current tag bearing", currentTagBearing);
            telemetry.addData("Target tag bearing", targetTagBearing);

            // this finds how far off we are from where we want to be
            // this is assuming you want to be facing directly towards the april tag (thus, the target is 0 degrees)
            double error = targetTagBearing - currentTagBearing;

            // this scales your error by a proportional constant so you can change how quickly it turns to face the tag
            error *= kP;

            telemetry.addData("Error", error);

            if (Math.abs(error) > 0.05) {
                front_left.setPower(-error);
                front_right.setPower(error);
                back_left.setPower(-error);
                back_right.setPower(error);
            }

            telemetry.update();
        }
    }   // end method runOpMode()

    @SuppressLint("DefaultLocale")
    private void aprilTagProcessing() {
        List<AprilTagDetection> currentDetections = processor.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            currentTagBearing = detection.ftcPose.bearing;
        }   // end for() loop

        if(currentDetections.isEmpty()) {
            // stop turning (because there is nothing to turn to)
            // this prevents the robot from spinning infinitely when it loses sight of the april tag
            currentTagBearing = targetTagBearing;
        }
    }   // end method telemetryAprilTag()


}   // end class

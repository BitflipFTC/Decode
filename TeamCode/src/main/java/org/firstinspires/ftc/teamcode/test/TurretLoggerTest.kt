package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.bylazar.battery.PanelsBattery
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import gay.zharel.fateweaver.flight.FlightRecorder
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.hardware.OV9281
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.hoodPosition
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.kD
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.kI
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.kP
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.kS
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.kV
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.lowPassCoeff
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.maxIntegral
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.minIntegral
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.rawPower
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.targetRPM
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.totPower
import org.firstinspires.ftc.teamcode.util.PIDController
import kotlin.math.pow
import kotlin.math.sqrt

@TeleOp(name = "Test: Turret Logger", group = "Test")
class TurretLoggerTest : LinearOpMode() {
    lateinit var camera: OV9281
    val flywheel by lazy { hardwareMap["flywheel"] as DcMotorEx }
    val hood by lazy { hardwareMap["hood"] as Servo }

    val controller = PIDController(kP, kI, kD, kV, kS, maxIntegral, minIntegral)
    val flywheelppr = 28
    var flywheelRPM = 0.0
    var lastFlywheelRPM = 0.0

    override fun runOpMode() {
        val timer = ElapsedTime()
        val timestampRecorder = FlightRecorder.createChannel("TIMESTAMP", Double::class.java)
        val hoodPositionRecorder = FlightRecorder.createChannel("HOOD POSITION", Double::class.java)
        val flywheelRPMRecorder = FlightRecorder.createChannel("FLYWHEEL RPM", Double::class.java)
        val distanceRecorder = FlightRecorder.createChannel("DISTANCE (IN)", Double::class.java)

        telemetry = JoinedTelemetry(
            PanelsTelemetry.ftcTelemetry,
            FtcDashboard.getInstance().telemetry,
            telemetry
        )

        camera = OV9281(this)

        flywheel.direction =
            DcMotorSimple.Direction.REVERSE // changed, switch this to forward and pidoutput to "-" if it isn't working (and velocity to -)
        flywheel.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        flywheel.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        flywheel.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT

        controller.setPointTolerance = 5.toDouble()

        hood.position = 0.0

        // bulk caching
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        allHubs.forEach { hub -> hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        waitForStart()

        timer.reset()
        hood.position = hoodPosition

        while (opModeIsActive()) {
            // more bulk caching
            allHubs.forEach { hub -> hub.clearBulkCache() }

            hood.position = hoodPosition

            targetRPM += gamepad1.left_stick_x

            //     currentRPM * 0.1 + lastRPM * 0.9
            flywheelRPM =
                ((flywheel.velocity / flywheelppr) * 60) * lowPassCoeff + (1 - lowPassCoeff) * lastFlywheelRPM
            lastFlywheelRPM = flywheelRPM

            controller.setCoeffs(kP, kI, kD, kV, kS)

            var pidOutput = controller.calculate(flywheelRPM, targetRPM)
            if (controller.error <= -750) pidOutput = 0.0 // prevents hard stops

            if (rawPower) {
                flywheel.power = totPower
            } else {
                flywheel.power = pidOutput
            }

            if (gamepad1.crossWasPressed()) {
                gamepad1.rumble(500)
                gamepad1.setLedColor(255.0, 136.0, 30.0, 500)
                
                timestampRecorder.put(timer.milliseconds())
                hoodPositionRecorder.put(hoodPosition)
                flywheelRPMRecorder.put(targetRPM)
                distanceRecorder.put(getDistanceToGoal())
            }

            telemetry.addData("Current RPM", flywheelRPM)
            telemetry.addData("Target RPM", targetRPM)
            telemetry.addData("Hood position", hoodPosition)
            telemetry.addData("Distance to april tag", getDistanceToGoal())
            telemetry.update()
        }
    }

    /**
     * @return -1.0 if there is no detected goal april tag. Otherwise, it returns the distance from the robot to the apriltag in inches.
     */
    fun getDistanceToGoal(): Double {
        val currentDetections = camera.aprilTag.detections

        if (!currentDetections.isEmpty()) {
            telemetry.addData("Detected april tags", currentDetections.size)
            var dist = -1.0

            currentDetections.forEach { detection ->
                {
                    telemetry.addData("TAG NAME", detection.metadata.name)

                    if (!detection.metadata.name.contains("Obelisk")) {
                        val tagPos = Pose2D(
                            DistanceUnit.INCH,
                            detection.metadata.fieldPosition.get(0).toDouble(),
                            detection.metadata.fieldPosition.get(1).toDouble(),
                            AngleUnit.DEGREES,
                            detection.metadata.fieldOrientation.toOrientation(
                                AxesReference.EXTRINSIC,
                                AxesOrder.XYZ,
                                AngleUnit.DEGREES
                            ).secondAngle.toDouble()
                        ) // given north is 0, clockwise

                        val robotPos = Pose2D(
                            detection.robotPose.getPosition().unit,
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            AngleUnit.DEGREES,
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)
                        )

                        dist = sqrt(
                            (tagPos.getX(DistanceUnit.INCH) - robotPos.getX(DistanceUnit.INCH)).pow(
                                2.0
                            ) + (tagPos.getY(DistanceUnit.INCH) - robotPos.getY(DistanceUnit.INCH)).pow(
                                2.0
                            )
                        )
                    }
                }
            }

            return dist
        } else { // no detections
            telemetry.addData("Detected april tags", 0)
            return -1.0
        }
    }

}
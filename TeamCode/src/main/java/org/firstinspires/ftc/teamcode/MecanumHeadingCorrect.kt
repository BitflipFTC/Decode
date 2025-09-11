package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.util.TunablePIDFController
import kotlin.math.abs

@Configurable
@Config
@TeleOp(name = "Mecanum Heading Correct")
class MecanumHeadingCorrect : LinearOpMode() {
    private val frontLeft  by lazy { hardwareMap["frontleft"]  as DcMotorEx }
    private val frontRight by lazy { hardwareMap["frontright"] as DcMotorEx }
    private val backLeft   by lazy { hardwareMap["backleft"]   as DcMotorEx }
    private val backRight  by lazy { hardwareMap["backright"]  as DcMotorEx }

    private val imu        by lazy { hardwareMap["imu"] as IMU }
    private var targetImuPos = 0.0

    @JvmField
    var p : Double = 0.005
    @JvmField
    var i : Double = 0.0
    @JvmField
    var d : Double = 0.0

    @JvmField
    var driveSpeed : Double = 0.5

    val controller = TunablePIDFController(p, i, d)

    override fun runOpMode() {
        telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)

        frontRight.direction = DcMotorSimple.Direction.REVERSE
        backRight.direction  = DcMotorSimple.Direction.REVERSE

        frontLeft.mode  = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        frontRight.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        backLeft.mode   = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        backRight.mode  = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        frontLeft.zeroPowerBehavior  = DcMotor.ZeroPowerBehavior.BRAKE
        frontRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        backLeft.zeroPowerBehavior   = DcMotor.ZeroPowerBehavior.BRAKE
        backRight.zeroPowerBehavior  = DcMotor.ZeroPowerBehavior.BRAKE

        frontLeft.mode  = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        frontRight.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        backLeft.mode   = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        backRight.mode  = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        imu.initialize(IMU.Parameters(RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
            RevHubOrientationOnRobot.UsbFacingDirection.UP)))

        targetImuPos = imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES)

        controller.setPointTolerance = 1.toDouble()

        // bulk caching
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        allHubs.forEach { hub -> hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        waitForStart()

        while (opModeIsActive()) {
            // more bulk caching
            allHubs.forEach { hub -> hub.clearBulkCache() }

            val x : Double = gamepad1.left_stick_x.toDouble()
            val y : Double = -gamepad1.left_stick_y.toDouble()
            val rot : Double = gamepad1.right_stick_x.toDouble()

            val heading = imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES)
            controller.setPID(p,i,d)
            val error = controller.calculate(heading, targetImuPos)

            var frontLeftPower = y + x - rot
            var frontRightPower = y - x + rot
            var backLeftPower = y - x - rot
            var backRightPower = y + x + rot

            // if turning, update the heading
            if (abs(rot - 0.0) > 0.01) {
                targetImuPos = heading
            } else {
                // otherwise, automatically steer to correct for drift
                frontLeftPower -= error
                frontRightPower += error
                backLeftPower -= error
                backRightPower += error
            }

            val max = maxOf(abs(frontLeftPower),abs(frontRightPower),abs(backLeftPower),abs(backRightPower))

            var usemax = false
            if (max > 1) {
                usemax = true

                frontLeftPower /= max
                frontRightPower /= max
                backLeftPower /= max
                backRightPower /= max
            }

            frontLeft.power  = frontLeftPower  * driveSpeed
            frontRight.power = frontRightPower * driveSpeed
            backLeft.power   = backLeftPower   * driveSpeed
            backRight.power  = backRightPower  * driveSpeed

            telemetry.addData("Using max", usemax)
            telemetry.addData("target imu position", targetImuPos)
            telemetry.addData("current Imu position", heading)
            telemetry.addLine("---------------------------------------")
            telemetry.addData("Front Left Wheel", frontLeftPower)
            telemetry.addData("Front Right Wheel", frontRightPower)
            telemetry.addData("Back Left Wheel", backLeftPower)
            telemetry.addData("Back Right Wheel", backRightPower)

            telemetry.update()
        }
    }
}

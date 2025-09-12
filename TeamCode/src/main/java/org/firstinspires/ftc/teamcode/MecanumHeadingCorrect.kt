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
import org.firstinspires.ftc.teamcode.util.PIDController
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

    val controller = PIDController(p,i,d)

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

        controller.setPointTolerance = 1.toDouble()

        imu.initialize(IMU.Parameters(RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
            RevHubOrientationOnRobot.UsbFacingDirection.UP)))

        targetImuPos = imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES)

        // bulk caching
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        allHubs.forEach { hub -> hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        waitForStart()

        while (opModeIsActive()) {
            // more bulk caching
            allHubs.forEach { hub -> hub.clearBulkCache() }

            val x : Double = gamepad1.left_stick_x.toDouble()
            val y : Double = -gamepad1.left_stick_y.toDouble()
            val rx : Double = gamepad1.right_stick_x.toDouble()

            val heading = imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES)
            
            controller.setCoeffs(p,i,d)
            val pidOutput : Double = controller.calculate(heading, targetImuPos)

            var frontLeftPower = y + x - rx
            var frontRightPower = y - x + rx
            var backLeftPower = y - x - rx
            var backRightPower = y + x + rx

            // if turning, update the heading
            if (abs(rx - 0.0) > 0.01) {
                targetImuPos = heading
            } else {
                // otherwise, automatically steer to correct for drift
                frontLeftPower -= pidOutput
                frontRightPower += pidOutput
                backLeftPower -= pidOutput
                backRightPower += pidOutput
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
            telemetry.addData("targ. Imu position", targetImuPos)
            telemetry.addData("curr. Imu position", heading)
            telemetry.addData("PID Output", pidOutput)
            telemetry.addLine("---------------------------------------")
            telemetry.addData("Front Left Wheel", frontLeftPower)
            telemetry.addData("Front Right Wheel", frontRightPower)
            telemetry.addData("Back Left Wheel", backLeftPower)
            telemetry.addData("Back Right Wheel", backRightPower)
            telemetry.addLine("---------------------------------------")
            telemetry.addData("Left_X",x)
            telemetry.addData("Left_Y",y)
            telemetry.addData("RightX",rx)
            telemetry.addLine("---------------------------------------")
            telemetry.addData("PID Controller variables", "")
            telemetry.addData("PID kP", controller.kP)
            telemetry.addData("PID kI", controller.kI)
            telemetry.addData("PID kD", controller.kD)
            telemetry.addData("PID Target Position", controller.targetPosition)
            telemetry.addData("PID Current Position", controller.currentPosition)
            telemetry.addData("PID Error", controller.error)
            telemetry.addData("PID Velocity error", controller.velError)
            telemetry.addData("PID Last Error", controller.lastError)
            telemetry.addData("PID SetPoint Tolerance", controller.setPointTolerance)
            telemetry.addData("PID Loop time", controller.timePeriod)
            telemetry.addData("PID Total Error", controller.totalError)
            telemetry.addData("PID At SetPoint", controller.atSetPoint())

            telemetry.update()
        }
    }
}

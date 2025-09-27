package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.bylazar.configurables.annotations.Configurable
import com.bylazar.gamepad.PanelsGamepad
import com.bylazar.graph.PanelsGraph
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.utils.LoopTimer
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.util.HeadingCorrectPID
import org.firstinspires.ftc.teamcode.util.HeadingCorrectPID.targetImuPos
import org.firstinspires.ftc.teamcode.util.PIDController
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

@TeleOp(name = "Mecanum Heading Correct")
class MecanumHeadingCorrect : LinearOpMode() {
    private val frontLeft  by lazy { hardwareMap["frontleft"]  as DcMotorEx }
    private val frontRight by lazy { hardwareMap["frontright"] as DcMotorEx }
    private val backLeft   by lazy { hardwareMap["backleft"]   as DcMotorEx }
    private val backRight  by lazy { hardwareMap["backright"]  as DcMotorEx }

    private val imu        by lazy { hardwareMap["imu"] as IMU }
    private var lastImuPos = 0.0

    private val loopTimer = LoopTimer()
    private var rotating = false

    private var fieldCentric : Boolean = false

    val ga1 = PanelsGamepad.firstManager

    var driveSpeed : Double = 0.5

    val controller = PIDController(HeadingCorrectPID.p,HeadingCorrectPID.i, HeadingCorrectPID.d)

    override fun runOpMode() {
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry, FtcDashboard.getInstance().telemetry)
        val graphManager = PanelsGraph.manager

        val g1 = PanelsGamepad.firstManager
        frontLeft.direction = DcMotorSimple.Direction.REVERSE
        backLeft.direction  = DcMotorSimple.Direction.REVERSE

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

        loopTimer.start()
        lastImuPos = imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES)

        while (opModeIsActive()) {
            // more bulk caching
            allHubs.forEach { hub -> hub.clearBulkCache() }
            val g1 = ga1.asCombinedFTCGamepad(gamepad1)

            var x : Double = gamepad1.left_stick_x.toDouble()
            var y : Double = -gamepad1.left_stick_y.toDouble()
            val rx : Double = gamepad1.right_stick_x.toDouble()

            val heading = imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES)
            val imuVelocity = heading - lastImuPos
            lastImuPos = heading
            if (abs(heading - targetImuPos) > 180) {
                controller.resetTotalError()

                if (heading > targetImuPos) {
                    targetImuPos += 360
                } else {
                    targetImuPos -= 360
                }
            }
            
            controller.setCoeffs(HeadingCorrectPID.p,HeadingCorrectPID.i,HeadingCorrectPID.d)
            val pidOutput : Double = -controller.calculate(heading, targetImuPos)

            if (gamepad1.left_trigger >= 0.25)
                fieldCentric = !fieldCentric


            if (fieldCentric) {
                var theta = atan2(y,x)
                val r = hypot(x,y)
                theta = AngleUnit.normalizeRadians(theta - imu.robotYawPitchRollAngles.getYaw(
                    AngleUnit.RADIANS))

                x = r * cos(theta)
                y = r * sin(theta)
            }

            if (gamepad1.yWasPressed()) {
                imu.resetYaw()
                targetImuPos = imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES)
            }
            var frontLeftPower = y + x + rx
            var frontRightPower = y - x - rx
            var backLeftPower = y - x + rx
            var backRightPower = y + x - rx

            // if turning, update the heading
            if (abs(rx - 0.0) > 0.01) {
                targetImuPos = heading
                controller.resetTotalError()
                rotating = true
            } else if (rotating) {
                if (abs(imuVelocity) > 0.1)
                    targetImuPos = heading
                else {
                    rotating = false
                }
            } else if (abs(pidOutput) >= 0.15) {
                // otherwise, automatically steer to correct for drift
                frontLeftPower += pidOutput
                frontRightPower -= pidOutput
                backLeftPower += pidOutput
                backRightPower -= pidOutput
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

            graphManager.addData("setpoint (target)", targetImuPos)
            graphManager.addData("Process variable (current)", heading)
            graphManager.addData("Heading velocity", imuVelocity)
            graphManager.update()

            telemetry.addData("targ. Imu position", targetImuPos)
            telemetry.addData("curr. Imu position", heading)
            telemetry.addData("IMU Velocity","%+05.2fdeg/s",imuVelocity)
            telemetry.addData("PID Output", pidOutput)
            telemetry.addLine("---------------------------------------")
            telemetry.addLine()

            telemetry.addData("","%+05.2f -------------------- %+05.2f", frontLeftPower,frontRightPower)
            telemetry.addLine("     |                                |")
            telemetry.addData("     |               ","%+05.3f       |",heading)
            telemetry.addLine("     |                                |")
            telemetry.addData("","%+05.2f -------------------- %+05.2f",backLeftPower,backRightPower)

            telemetry.addLine()

            telemetry.addLine("---------------------------------------")
            telemetry.addData("Left Stick ","x:%+05.2f y:%+05.2f", x, y)
            telemetry.addData("Right Stick","x:%+05.2f y:%+05.2f", rx, gamepad1.right_stick_y)
            telemetry.addLine("---------------------------------------")
            telemetry.addData("PID Controller variables", "kP:%.3f  kI:%.3f  kD:%.3f",
                HeadingCorrectPID.p, HeadingCorrectPID.i, HeadingCorrectPID.d)
            telemetry.addData("PID Error", controller.error)
            telemetry.addData("PID Velocity error", controller.velError)
            telemetry.addData("PID SetPoint Tolerance", controller.setPointTolerance)
            telemetry.addData("PID Total Error", controller.totalError)
            telemetry.addData("PID At SetPoint", controller.atSetPoint())
            telemetry.addLine("---------------------------------------")
            telemetry.addData("Loop Time", loopTimer.ms)


            telemetry.update()
        }
    }
}

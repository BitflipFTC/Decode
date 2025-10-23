package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.bylazar.gamepad.PanelsGamepad
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
import org.firstinspires.ftc.teamcode.util.HeadingCorrectPID.kD
import org.firstinspires.ftc.teamcode.util.HeadingCorrectPID.kI
import org.firstinspires.ftc.teamcode.util.HeadingCorrectPID.kP
import org.firstinspires.ftc.teamcode.util.HeadingCorrectPID.targetImuPos
import org.firstinspires.ftc.teamcode.util.PIDController
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

@TeleOp(name = "Heading Correct Drive", group = "TeleOp")
class HeadingCorrectDrive : LinearOpMode() {
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

    val headingController = PIDController(kP,kI, kD)

//    val flywheelController = PIDController(
//        FlywheelTestPID.kP,
//        FlywheelTestPID.kI,
//        FlywheelTestPID.kD, kV, kS,maxIntegral, minIntegral)
//    val flywheel by lazy { hardwareMap["flywheel"] as DcMotorEx }
//    val flywheelppr = 28
//    val hood by lazy { hardwareMap["hood"] as Servo }
//
//    var flywheelRPM = 0.0
//    var lastFlywheelRPM = 0.0
//
//    var hoodPos = 0.0

    override fun runOpMode() {
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry, FtcDashboard.getInstance().telemetry)
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

        headingController.setPointTolerance = 1.toDouble()

        imu.initialize(IMU.Parameters(RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT)))

        targetImuPos = imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES)



//        flywheel.direction = DcMotorSimple.Direction.FORWARD
//        flywheel.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
//        flywheel.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
//        flywheel.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
//
//        flywheelController.setPointTolerance = 5.toDouble()
//        flywheelController.setpoint = 3500.0
//
//        hood.position = hoodPos


        // bulk caching
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        allHubs.forEach { hub -> hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        waitForStart()

        loopTimer.start()
        lastImuPos = imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES)

        while (opModeIsActive()) {
            // more bulk caching
            allHubs.forEach { hub -> hub.clearBulkCache() }
            ga1.asCombinedFTCGamepad(gamepad1)

            var x : Double = gamepad1.left_stick_x.toDouble()
            var y : Double = -gamepad1.left_stick_y.toDouble()
            val rx : Double = gamepad1.right_stick_x.toDouble()

            val heading = imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES)
            val imuVelocity = heading - lastImuPos
            lastImuPos = heading
            if (abs(heading - targetImuPos) > 180) {
                headingController.resetTotalError()

                if (heading > targetImuPos) {
                    targetImuPos += 360
                } else {
                    targetImuPos -= 360
                }
            }

            headingController.setCoeffs(kP,kI,kD)
            val pidOutput : Double = -headingController.calculate(heading, targetImuPos)

            if (gamepad1.left_trigger >= 0.25)
                fieldCentric = !fieldCentric

            if (fieldCentric) {
                // impl 1:
//                var theta = atan2(y,x)
//                val r = hypot(x,y)
//                theta = AngleUnit.normalizeRadians(theta - imu.robotYawPitchRollAngles.getYaw(
//                    AngleUnit.RADIANS))
//
//                x = r * cos(theta)
//                y = r * sin(theta)

                // impl 2:
                x = x * cos(Math.toRadians(heading)) - y * sin(Math.toRadians(heading))
                y = x * sin(Math.toRadians(heading)) + y * sin(Math.toRadians(heading))
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
                headingController.resetTotalError()
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
//
//            val max = maxOf(abs(frontLeftPower),abs(frontRightPower),abs(backLeftPower),abs(backRightPower))
//
//            var usemax = false
//            if (max > 1) {
//                usemax = true
//
//                frontLeftPower /= max
//                frontRightPower /= max
//                backLeftPower /= max
//                backRightPower /= max
//            }

            frontLeft.power  = frontLeftPower  * driveSpeed
            frontRight.power = frontRightPower * driveSpeed
            backLeft.power   = backLeftPower   * driveSpeed
            backRight.power  = backRightPower  * driveSpeed

//            hoodPos += (gamepad1.right_bumper.toInt() * 0.01 - gamepad1.left_bumper.toInt() * 0.01)
//            hood.position = hoodPos
//
//            flywheelRPM = (-(flywheel.velocity / flywheelppr) * 60) * lowPassCoeff + (1 - lowPassCoeff) * lastFlywheelRPM
//            lastFlywheelRPM = flywheelRPM
//            val flywheelPower = flywheelController.calculate(flywheelRPM, targetRPM)
//            flywheel.power = flywheelPower

//            telemetry.addData("Current RPM", flywheelRPM)
//            telemetry.addData("Target RPM", targetRPM)
//            telemetry.addData("flywheel Power", flywheelPower)
//            telemetry.addData("battery", PanelsBattery.provider.batteryVoltage)


            telemetry.addData("targ. Imu position", targetImuPos)
            telemetry.addData("curr. Imu position", heading)
            telemetry.addData("IMU Velocity","%+05.2fdeg/s",imuVelocity)
            telemetry.addData("heading PID Output", pidOutput)
            telemetry.addLine("---------------------------------------")
            telemetry.addData("PID Error", headingController.error)
            telemetry.addData("PID At SetPoint", headingController.atSetPoint())
            telemetry.addLine("---------------------------------------")




            telemetry.update()
        }
    }
}

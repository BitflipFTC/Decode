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
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.kD
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.kV
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.kI
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.kP
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.maxIntegral
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.minIntegral
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.targetRPM
import org.firstinspires.ftc.teamcode.util.PIDController
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.hoodPosition
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.kS
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.lowPassCoeff
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.rawPower
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.totPower

@TeleOp(name = "Test: Flywheel", group = "Test")
class FlywheelTest : LinearOpMode() {
    val controller = PIDController(kP, kI, kD, kV, kS,maxIntegral, minIntegral)
    val flywheel by lazy { hardwareMap["flywheel"] as DcMotorEx }
    val flywheelppr = 28
    val hood by lazy { hardwareMap["hood"] as Servo }

    var flywheelRPM = 0.0
    var lastFlywheelRPM = 0.0

    override fun runOpMode() {
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, FtcDashboard.getInstance().telemetry, telemetry)

        flywheel.direction = DcMotorSimple.Direction.FORWARD
        flywheel.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        flywheel.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        flywheel.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT

        controller.setPointTolerance = 5.toDouble()


        // bulk caching
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        allHubs.forEach { hub -> hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        waitForStart()
        hood.position = hoodPosition

        while (opModeIsActive()) {
            // more bulk caching
            allHubs.forEach { hub -> hub.clearBulkCache() }

            // reset the target rpm
            if (gamepad1.aWasPressed()) {
                targetRPM = 4500.0
            }

            hood.position = hoodPosition

            targetRPM += gamepad1.left_stick_x

            //     currentRPM * 0.1 + lastRPM * 0.9
            flywheelRPM = (-(flywheel.velocity / flywheelppr) * 60) * lowPassCoeff + (1 - lowPassCoeff) * lastFlywheelRPM
            lastFlywheelRPM = flywheelRPM

            controller.setCoeffs(kP, kI, kD, kV,kS)

            var pidOutput = -controller.calculate(flywheelRPM, targetRPM)

            if(controller.error <= -750) pidOutput = 0.0
            if (rawPower) {
                flywheel.power = totPower
            } else {
                flywheel.power = pidOutput
            }

            telemetry.addData("Current RPM", flywheelRPM)
            telemetry.addData("Target RPM", targetRPM)
            telemetry.addData("Motor Power", pidOutput)
            telemetry.addData("battery", PanelsBattery.provider.batteryVoltage)
            telemetry.update()
        }
    }

}
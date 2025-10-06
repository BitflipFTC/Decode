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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.kD
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.kV
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.kI
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.kP
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.maxIntegral
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.minIntegral
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.targetRPM
import org.firstinspires.ftc.teamcode.util.PIDController
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.hoodangle
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.kS
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.rawPower
import org.firstinspires.ftc.teamcode.util.FlywheelTestPID.totPower

@TeleOp(name = "Test: Flywheel", group = "Test")
class FlywheelTest : LinearOpMode() {
    val controller = PIDController(kP, kI, kD, kV, kS,maxIntegral, minIntegral)
    val flywheel by lazy { hardwareMap["flywheel"] as DcMotorEx }
    val hood by lazy { hardwareMap["hood"] as Servo }

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
        hood.position = hoodangle

        while (opModeIsActive()) {
            // more bulk caching
            allHubs.forEach { hub -> hub.clearBulkCache() }

            // reset the target rpm
            if (gamepad1.aWasPressed()) {
                targetRPM = 4500.0
            }
            hood.position = hoodangle


            targetRPM += gamepad1.left_stick_x

            //                         degrees/s / 360 = rotations/s   rotations/s * 60 = rotations/m
            val flywheelRPM = -(flywheel.getVelocity(AngleUnit.DEGREES) / 360) * 6000
//
            controller.setCoeffs(kP, kI, kD, kV,kS)
            val pidOutput = controller.calculate(flywheelRPM, targetRPM)
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
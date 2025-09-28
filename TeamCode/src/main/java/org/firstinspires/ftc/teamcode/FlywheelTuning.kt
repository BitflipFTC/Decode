package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.util.FlywheelVelPID.kD
import org.firstinspires.ftc.teamcode.util.FlywheelVelPID.kF
import org.firstinspires.ftc.teamcode.util.FlywheelVelPID.kI
import org.firstinspires.ftc.teamcode.util.FlywheelVelPID.kP
import org.firstinspires.ftc.teamcode.util.FlywheelVelPID.maxIntegral
import org.firstinspires.ftc.teamcode.util.FlywheelVelPID.minIntegral
import org.firstinspires.ftc.teamcode.util.FlywheelVelPID.targetRPM
import org.firstinspires.ftc.teamcode.util.PIDController

@TeleOp(name = "Tuning: Flywheel", group = "Tuning")
class FlywheelTuning : LinearOpMode() {
    val controller = PIDController(kP, kI, kD, kF, maxIntegral, minIntegral)
    val flywheel by lazy { hardwareMap["flywheel"] as DcMotorEx }

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

        while (opModeIsActive()) {
            // more bulk caching
            allHubs.forEach { hub -> hub.clearBulkCache() }

            if (gamepad1.aWasPressed()) {
                targetRPM = 3000.0
            }

            targetRPM += gamepad1.left_stick_x

            //                         degrees/s / 360 = rotations/s   rotations/s * 60 = rotations/m
            val flywheelRPM = (flywheel.getVelocity(AngleUnit.DEGREES) / 360) * 60

            controller.setCoeffs(kP, kI, kD, kF)
            val pidOutput = controller.calculate(flywheelRPM, targetRPM)
            flywheel.power = pidOutput

            telemetry.addData("Current RPM", flywheelRPM)
            telemetry.addData("Target RPM", targetRPM)
            telemetry.addData("Motor Power", pidOutput)
            telemetry.update()
        }
    }

}
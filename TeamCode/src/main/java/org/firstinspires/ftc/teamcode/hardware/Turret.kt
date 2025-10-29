package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.util.PIDController

@Config
class Turret(opMode: OpMode) {
    companion object {
        @JvmField
        var kP = 0.001
        @JvmField
        var kI = 0.0
        @JvmField
        var kD = 0.0
        @JvmField
        var kS = 0.0275
        @JvmField
        var setPointTolerance : Double = 3.toDouble() // degrees
    }

    val hwMap: HardwareMap = opMode.hardwareMap
    val telemetry: Telemetry = opMode.telemetry

    val servoL : CRServoImplEx by lazy { hwMap["turretL"] as CRServoImplEx }
    val servoR : CRServoImplEx by lazy { hwMap["turretR"] as CRServoImplEx }
    val controller = PIDController(kP, kI, kD, 0.0, kS)

    var pidOutput: Double = 0.0
    init {
        // defaults to 600,2400 - this gives full range
        servoR.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        servoL.pwmRange = PwmControl.PwmRange(500.0, 2500.0)

        controller.setPointTolerance = setPointTolerance
    }

    fun setPower (pow : Double) {
        servoR.power = pow
        servoL.power = pow
    }

    fun periodic(tagBearing: Double) {
        pidOutput = controller.calculate(tagBearing, 0.0) // bearing approaches 0

        setPower(pidOutput)
    }

    fun getPower() = pidOutput

    fun atSetPoint() = controller.atSetPoint()
}
package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import org.firstinspires.ftc.teamcode.util.PIDController
import org.firstinspires.ftc.teamcode.util.Subsystem

@Config
class Turret(val hwMap : HardwareMap) {
    companion object {
        @JvmField
        var kP = 0.0
        @JvmField
        var kI = 0.0
        @JvmField
        var kD = 0.0
        @JvmField
        var kS = 0.0
    }
    val servoL : CRServoImplEx by lazy { hwMap["turretR"] as CRServoImplEx }
    val servoR : CRServoImplEx by lazy { hwMap["turretL"] as CRServoImplEx }
    val controller = PIDController(kP, kI, kD, 0.0, kS)

    init {
        // defaults to 600,2400 - this gives full range
        servoR.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        servoL.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
    }

    fun setPower (pow : Double) {
        servoR.power = pow
        servoL.power = pow
    }
}
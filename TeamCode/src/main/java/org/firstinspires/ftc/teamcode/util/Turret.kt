package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl

class Turret(val hwMap : HardwareMap) {
    val servoL : CRServoImplEx by lazy { hwMap["turretR"] as CRServoImplEx }
    val servoR : CRServoImplEx by lazy { hwMap["turretL"] as CRServoImplEx }

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
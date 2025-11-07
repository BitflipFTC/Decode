package org.firstinspires.ftc.teamcode.util.hardware

import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.PwmControl
import dev.nextftc.ftc.ActiveOpMode
import kotlin.math.abs

class CRServoEx(
    servoName: String,
    var cacheTolerance: Double = 0.01
) {
    private val crServo = ActiveOpMode.hardwareMap.get(CRServoImplEx::class.java, servoName).apply {
        pwmRange = PwmControl.PwmRange(500.0, 2500.0)
    }

    private var lastPower = 0.0
    var power: Double
        get() = crServo.power
        set(power) {
            if (abs(power - lastPower) > cacheTolerance) {
                crServo.power = power
            }
        }
}
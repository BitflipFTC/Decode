package org.firstinspires.ftc.teamcode.util.hardware

import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.PwmControl
import kotlin.math.abs

/**
 * @param servoName the configured name of the servo on the RobotController
 * @param cacheTolerance the range in which a new power won't be sent
 */
class CRServoEx(
    servoName: String,
    var cacheTolerance: Double = 0.01
) {
    private val crServo = ActiveOpMode.hardwareMap.get(CRServoImplEx::class.java, servoName).apply {
        pwmRange = PwmControl.PwmRange(500.0, 2500.0)
    }

    private var lastPower = 0.0

    /**
     * @param power the power to set the servo to
     */
    var power: Double
        get() = crServo.power
        set(power) {
            lastPower = crServo.power
            if (abs(power - lastPower) > cacheTolerance) {
                crServo.power = power
            }
        }
}
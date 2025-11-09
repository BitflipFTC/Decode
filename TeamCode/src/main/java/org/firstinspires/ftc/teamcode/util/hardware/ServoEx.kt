package org.firstinspires.ftc.teamcode.util.hardware

import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import dev.nextftc.ftc.ActiveOpMode
import kotlin.math.abs

/**
 * @param servoName the configured name of the servo on the RobotController
 * @param cacheTolerance the range in which a new position won't be sent
 */
class ServoEx(
    servoName: String,
    var cacheTolerance: Double = 0.01
) {
    private val servo: ServoImplEx =
        ActiveOpMode.hardwareMap.get(ServoImplEx::class.java, servoName).apply {
            pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        }

    private var lastPosition = 0.0

    /**
     * @param position the position to set the servo to
     */
    var position: Double
        get() = servo.position
        set(position) {
            lastPosition = servo.position
            if (abs(position - lastPosition) > cacheTolerance) {
                servo.position = position
            }
        }
}
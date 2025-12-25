package org.firstinspires.ftc.teamcode.util.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.util.OpModeConstants.hardwareMap
import kotlin.math.abs

/**
 * @param motorName the configured name of the motor on the RobotController
 * @param cacheTolerance the range in which a new position won't be sent
 * @param maxSlewRate the maximum change in motor power per loop
 */
class MotorEx @JvmOverloads constructor(
    motorName: String,
    var cacheTolerance: Double = 0.01,
    var maxSlewRate: Double = Double.POSITIVE_INFINITY
) {
    private val motor: DcMotorEx = hardwareMap!!.get(DcMotorEx::class.java, motorName)

    private var lastPower = 0.0
    var power: Double
        get() = motor.power
        set(power) {
            lastPower = motor.power
            if (abs(power - lastPower) > cacheTolerance) {
                val desiredChange = power - lastPower
                val limitedChange = Range.clip(desiredChange, -maxSlewRate, maxSlewRate)
                motor.power += limitedChange
            }
        }
    val currentPosition: Int
        get() = motor.currentPosition
    val velocity: Double
        get() = motor.velocity

    /**
     * Stops the motor and resets the encoder.
     * @return `this` - for method chaining
     */
    fun zeroed(): MotorEx {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        return this
    }

    /**
     * Sets the motor to FLOAT.
     * @return `this` - for method chaining
     */
    fun float(): MotorEx {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        return this
    }

    /**
     * Sets the motor to BRAKE.
     * @return `this` - for method chaining
     */
    fun brake(): MotorEx {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        return this
    }

    /**
     * Reverses the motor.
     * @return `this` - for method chaining
     */
    fun reverse(): MotorEx {
        motor.direction = DcMotorSimple.Direction.REVERSE
        return this
    }
}
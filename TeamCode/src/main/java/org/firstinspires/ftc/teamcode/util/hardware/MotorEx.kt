package org.firstinspires.ftc.teamcode.util.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.nextftc.ftc.ActiveOpMode
import kotlin.math.abs

class MotorEx(
    motorName: String,
    var cacheTolerance: Double = 0.01
) {
    private val motor: DcMotorEx = ActiveOpMode.hardwareMap.get(DcMotorEx::class.java, motorName)

    private var lastPower = 0.0
    var power: Double
        get() = motor.power
        set(power) {
            if (abs(power - lastPower) > cacheTolerance) {
                motor.power = power
            }
        }
    val currentPosition: Int
        get() = motor.currentPosition
    val velocity: Double
        get() = motor.velocity

    fun zeroed(): MotorEx {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        return this
    }

    fun float(): MotorEx {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        return this
    }

    fun brake(): MotorEx {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        return this
    }

    fun reverse(): MotorEx {
        motor.direction = DcMotorSimple.Direction.REVERSE
        return this
    }
}
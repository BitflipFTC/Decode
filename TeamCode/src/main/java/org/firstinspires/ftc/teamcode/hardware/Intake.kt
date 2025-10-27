package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

class Intake(val hwMap: HardwareMap) {
    private val motor by lazy { hwMap["intake"] as DcMotorEx }
    var power: Double
        get() = motor.power
        set(power) { motor.power = power }

    init {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor.direction = DcMotorSimple.Direction.REVERSE
    }

    fun toggle () {
        power = if (power == 0.0) 1.0 else 0.0
    }
}
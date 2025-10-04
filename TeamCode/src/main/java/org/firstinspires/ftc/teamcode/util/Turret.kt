package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap

class Turret(val hwMap : HardwareMap) {
    val servoL : CRServo by lazy { hwMap["turretR"] as CRServo }
    val servoR : CRServo by lazy { hwMap["turretL"] as CRServo }

    fun setPower (pow : Double) {
        servoR.power = pow
        servoL.power = pow
    }
}
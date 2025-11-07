package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.Gamepad

fun Boolean.toDouble(): Double {
    return if (this) 1.0 else 0.0
}

enum class Artifact {
    GREEN,
    PURPLE,
    NONE;

    fun firstLetter() = this.name.first()
}

enum class MotifPattern {
    GPP,
    PGP,
    PPG,
    NONE
}

fun Gamepad.dpadToAxes(): Pair<Double, Double> {
    val x = (this.dpadRightWasPressed().toDouble()) - (this.dpadLeftWasPressed().toDouble())
    val y = (this.dpadUpWasPressed().toDouble()) - (this.dpadDownWasPressed().toDouble())

    return Pair(x, y)
}
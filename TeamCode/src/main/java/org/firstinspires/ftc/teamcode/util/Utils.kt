package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.Gamepad

fun Boolean.toDouble(): Double {
    return if (this) 1.0 else 0.0
}

enum class Artifact {
    GREEN,
    PURPLE;

    fun firstLetter() = this.name.first()
}

enum class MotifPattern {
    GPP,
    PGP,
    PPG
}

enum class Alliance (val aprilTagID: Int) {
    RED(24),
    BLUE(20)
}

enum class StartingPosition () {
    GOAL,
    FAR
}

fun Gamepad.dpadToAxes(): Pair<Double, Double> {
    val x = (this.dpadRightWasPressed().toDouble()) - (this.dpadLeftWasPressed().toDouble())
    val y = (this.dpadUpWasPressed().toDouble()) - (this.dpadDownWasPressed().toDouble())

    return Pair(x, y)
}
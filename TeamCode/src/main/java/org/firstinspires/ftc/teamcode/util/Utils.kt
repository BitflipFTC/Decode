package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.hardware.OV9281
import org.openftc.apriltag.AprilTagDetection

fun Boolean.toInt(): Int {
    return if(this) 1 else 0
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

fun Gamepad.dpadToInts(): Pair<Int, Int> {
    val x = (this.dpadRightWasPressed().toInt() * 1) - (this.dpadLeftWasPressed().toInt())
    val y = (this.dpadUpWasPressed().toInt() * 1) - (this.dpadDownWasPressed().toInt())

    return Pair(x, y)
}
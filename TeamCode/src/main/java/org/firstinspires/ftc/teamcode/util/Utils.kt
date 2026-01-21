package org.firstinspires.ftc.teamcode.util

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathBuilder
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

fun Gamepad.dpadToAxes(): Pair<Double, Double> {
    val x = (this.dpadRightWasPressed().toDouble()) - (this.dpadLeftWasPressed().toDouble())
    val y = (this.dpadUpWasPressed().toDouble()) - (this.dpadDownWasPressed().toDouble())

    return Pair(x, y)
}

fun PathBuilder.buildBasicLine(p1: Pose, p2: Pose): PathBuilder = this
    .addPath(BezierLine(p1, p2))
    .setLinearHeadingInterpolation(p1.heading, p2.heading)

fun PathBuilder.buildCurvedLine(p1: Pose, c: Pose, p2: Pose): PathBuilder = this
    .addPath(BezierCurve(p1, c, p2))
    .setLinearHeadingInterpolation(p1.heading, p2.heading)

fun PathBuilder.buildTangentLine(p1: Pose, p2: Pose): PathBuilder = this
    .addPath(BezierLine(p1, p2))
    .setTangentHeadingInterpolation()
package org.firstinspires.ftc.teamcode.util

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathBuilder
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.util.auto.Path

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

fun PathBuilder.buildCurvedTangentLine(p1: Pose, c: Pose, p2: Pose): PathBuilder = this
    .addPath(BezierCurve(p1, c, p2))
    .setTangentHeadingInterpolation()

fun PathBuilder.buildTangentLine(p1: Pose, p2: Pose): PathBuilder = this
    .addPath(BezierLine(p1, p2))
    .setTangentHeadingInterpolation()

fun Follower.followCustomPath(path: Path) = this.followPath(path.path, path.speed, true)

/**
 * @author WPlib i think (copied from solverslib)
 */
fun normalizeAngle(angle: Double, zeroToMax: Boolean, angleUnit: AngleUnit): Double {
    val max: Double = returnMaxForAngleUnit(angleUnit)
    val angle2 = angle % max
    if (zeroToMax && angle2 < 0) {
        return angle2 + max
    } else if (!zeroToMax) {
        if (angle2 > max / 2) {
            return angle2 - max
        } else if (angle2 < -max / 2) {
            return angle2 + max
        }
    }

    return angle2
}

fun normalizeRadians(angle: Double, zeroToFull: Boolean): Double {
    return normalizeAngle(angle, zeroToFull, AngleUnit.RADIANS)
}

fun normalizeDegrees(angle: Double, zeroToFull: Boolean): Double {
    return normalizeAngle(angle, zeroToFull, AngleUnit.DEGREES)
}

fun returnMaxForAngleUnit(angleUnit: AngleUnit): Double {
    return if (angleUnit == AngleUnit.RADIANS) {
        Math.PI * 2
    } else {
        360.0
    }
}
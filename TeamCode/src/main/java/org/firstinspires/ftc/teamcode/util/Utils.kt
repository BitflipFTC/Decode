package org.firstinspires.ftc.teamcode.util

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

fun getMotif(camera: OV9281): MotifPattern {
    val currentDetections = camera.aprilTag.detections
    val obeliskDetections = currentDetections.filter { it.metadata.name.contains("Obelisk") }
    var pattern: MotifPattern = MotifPattern.NONE
    for (detection in obeliskDetections) {
        when (detection.metadata.name) {
            "Obelisk_GPP" -> pattern = MotifPattern.GPP
            "Obelisk_PGP" -> pattern = MotifPattern.PGP
            "Obelisk_PPG" -> pattern = MotifPattern.PPG
            else          -> {}
        }
    }

    return pattern
}
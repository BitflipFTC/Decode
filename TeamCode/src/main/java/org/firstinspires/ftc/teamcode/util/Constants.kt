package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.config.Config
import com.bylazar.configurables.annotations.Configurable

// TUNED
@Configurable
@Config
object HeadingCorrectPID {
    @JvmField var kP: Double = 0.022

    @JvmField var kI: Double = 0.11

    @JvmField var kD: Double = 0.0015
    @JvmField var targetImuPos : Double = 0.0
}

@Configurable
@Config
object AprilTagDriverPID {
    @JvmField var kP: Double = 0.0025

    @JvmField var kI: Double = 0.0

    @JvmField var kD: Double = 0.0

    @JvmField var min : Double = -10.0
    @JvmField var max : Double = 10.0
}

@Configurable
@Config
object AprilTagAutoPID {
    @JvmField var kP: Double = 0.011

    @JvmField var kI: Double = 0.0

    @JvmField var kD: Double = 0.000003

    @JvmField var min : Double = -10.0
    @JvmField var max : Double = 10.0
    @JvmField var targetTagPos : Double = 320.0
}

@Configurable
@Config
object FlywheelTestPID {
    @JvmField var kP : Double = 0.0007
    @JvmField var kI : Double = 0.0
    @JvmField var kD : Double = 0.0
    @JvmField var kV : Double = 0.00018
    @JvmField var kS : Double = 0.0

    @JvmField var maxIntegral: Double = 1.0
    @JvmField var minIntegral: Double = -1.0
    @JvmField var targetRPM : Double = 3000.0
    @JvmField var hoodPosition : Double = 0.3
    @JvmField var totPower : Double = 0.0
    @JvmField var rawPower : Boolean = false

    @JvmField var lowPassCoeff : Double = 0.1
}

fun Boolean.toInt(): Int {
    return if(this) 1 else 0
}

@Configurable
@Config
object ArtifactTrackAutoPID {
    @JvmField var kP : Double = 0.0
    @JvmField var kS : Double = 0.0
    @JvmField var kI : Double = 0.0
    @JvmField var kD : Double = 0.0
    @JvmField var kV : Double = 0.0
    @JvmField var maxIntegral: Double = 1.0
    @JvmField var minIntegral: Double = -1.0
    @JvmField var setPointTolerance : Double = 1.toDouble()

    @JvmField var minSize = 50.0
    @JvmField var maxSize = 20000.0
    @JvmField var minCirc = 0.6
    @JvmField var maxCirc = 1.0
}

enum class Artifact {
    GREEN,
    PURPLE,
    NONE;

    fun firstLetter() = this.name.first()
}
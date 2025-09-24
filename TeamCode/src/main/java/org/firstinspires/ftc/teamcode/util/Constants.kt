package org.firstinspires.ftc.teamcode.util

import com.bylazar.configurables.annotations.Configurable

@Configurable
object HeadingCorrectPID {
    @JvmField var p: Double = 0.022

    @JvmField var i: Double = 0.1

    @JvmField var d: Double = 0.0
}

@Configurable
object AprilTagDriverPID {
    @JvmField var p: Double = 0.0025

    @JvmField var i: Double = 0.0

    @JvmField var d: Double = 0.0

    @JvmField var min : Double = -10.0
    @JvmField var max : Double = 10.0
}

@Configurable
object AprilTagTrackerPID {
    @JvmField var p: Double = 0.000

    @JvmField var i: Double = 0.0

    @JvmField var d: Double = 0.0

    @JvmField var min : Double = -10.0
    @JvmField var max : Double = 10.0
}
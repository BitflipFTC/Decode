package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.config.Config

@Config
object HeadingCorrectPID {
    @JvmField var p: Double = 0.022

    @JvmField var i: Double = 0.1

    @JvmField var d: Double = 0.0
}

@Config
object AprilTagDriverPID {
    @JvmField var p: Double = 0.0025

    @JvmField var i: Double = 0.0

    @JvmField var d: Double = 0.0

    @JvmField var min : Double = -10.0
    @JvmField var max : Double = 10.0
}

@Config
object AprilTagTrackerPID {
    @JvmField var p: Double = 0.000

    @JvmField var i: Double = 0.0

    @JvmField var d: Double = 0.0

    @JvmField var min : Double = -10.0
    @JvmField var max : Double = 10.0
}
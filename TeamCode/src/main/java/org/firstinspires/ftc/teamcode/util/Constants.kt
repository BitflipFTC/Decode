package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.config.Config
import com.bylazar.configurables.annotations.Configurable

// TUNED
@Configurable
@Config
object HeadingCorrectPID {
    @JvmField var p: Double = 0.022

    @JvmField var i: Double = 0.11

    @JvmField var d: Double = 0.0015
    @JvmField var targetImuPos : Double = 0.0
}

@Configurable
@Config
object AprilTagDriverPID {
    @JvmField var p: Double = 0.0025

    @JvmField var i: Double = 0.0

    @JvmField var d: Double = 0.0

    @JvmField var min : Double = -10.0
    @JvmField var max : Double = 10.0
}

@Configurable
@Config
object AprilTagAutoPID {
    @JvmField var p: Double = 0.011

    @JvmField var i: Double = 0.0

    @JvmField var d: Double = 0.000003

    @JvmField var min : Double = -10.0
    @JvmField var max : Double = 10.0
    @JvmField var targetTagPos : Double = 320.0
}

// tuned (or so)
@Configurable
@Config
object FlywheelTestPID {
    @JvmField var kP : Double = 0.0000
    @JvmField var kI : Double = 0.0
    @JvmField var kD : Double = 0.0
    @JvmField var kF : Double = 0.00001

    @JvmField var maxIntegral: Double = 1.0
    @JvmField var minIntegral: Double = -1.0
    @JvmField var targetRPM : Double = 5500.0
    @JvmField var hoodangle : Double = 0.0
    @JvmField var totPower : Double = 0.0
    @JvmField var rawPower : Boolean = false
}

@Configurable
@Config
object TurretTestPID {
    @JvmField var kP : Double = 0.0
    @JvmField var kI : Double = 0.0
    @JvmField var kD : Double = 0.0
    @JvmField var kF : Double = 0.0
    @JvmField var maxIntegral: Double = 1.0
    @JvmField var minIntegral: Double = -1.0
    @JvmField var setPointTolerance : Double = 5.0
}
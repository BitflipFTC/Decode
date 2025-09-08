package org.firstinspires.ftc.teamcode.util

import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward

class TunablePIDFController @JvmOverloads constructor(
    kP: Double,
    kI: Double = 0.0,
    kD: Double = 0.0,
    kF: StaticFeedforward = StaticFeedforward(0.0),
    setPointTolerance: Double = 10.0
) : PIDFController(kP, kI, kD, kF, setPointTolerance) {

    fun setPID(p: Double, i: Double, d: Double) {
        this.kP = p
        this.kI = i
        this.kD = d
    }
}

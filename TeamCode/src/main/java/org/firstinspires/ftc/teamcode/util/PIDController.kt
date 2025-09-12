package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

class PIDController(
    var kP : Double = 0.0,
    var kI : Double = 0.0,
    var kD : Double = 0.0,
    var kF : Double = 0.0,
    private var maxIntegral: Double = 1.0,
    private var minIntegral: Double = -1.0
) {
    var targetPosition : Double = 0.0
    var currentPosition : Double = 0.0
        private set
    var setPointTolerance : Double = 0.0

    private val timer = ElapsedTime()
    private var lastTime = 0.0
    var timePeriod : Double = 0.0
        private set
    var lastError : Double = 0.0
        private set
    var totalError = 0.0
        private set


    fun setCoeffs (kP : Double = 0.0, kI: Double = 0.0, kD: Double = 0.0, kf: Double = 0.0) {
        this.kP = kP
        this.kI = kI
        this.kD = kD
        this.kF = kf
    }

    fun calculate (currentPosition : Double, targetPosition : Double = this.targetPosition) : Double {
        // handle time period stuff
        val currentTime = timer.nanoseconds() / 1E9
        if (lastTime == 0.0) lastTime = currentTime
        timePeriod = currentTime - lastTime

        lastTime = currentTime


        // handle error and velocity error
        this.currentPosition = currentPosition
        this.targetPosition = targetPosition

        val error = targetPosition - currentPosition
        val velError = if (timePeriod != 0.0) {(error - lastError) / timePeriod} else {0.0}
        lastError = error

        // handle integral error
        totalError += timePeriod * error
        totalError = min(maxIntegral, max(minIntegral, totalError))

        return kP * error + kI * totalError + kD * velError + targetPosition * kF
    }

    fun reset () {
        targetPosition = 0.0
        setPointTolerance = 0.0
        lastTime = 0.0
        timer.reset()
        totalError = 0.0
        lastError = 0.0
    }

    fun setIntegrationBounds (min : Double, max :Double) {
        minIntegral = min
        maxIntegral = max
    }

    fun atSetPoint () : Boolean {
        return (abs(targetPosition - currentPosition) <= setPointTolerance)
    }
}
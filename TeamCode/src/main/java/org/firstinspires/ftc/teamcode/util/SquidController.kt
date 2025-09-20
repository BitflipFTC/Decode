package org.firstinspires.ftc.teamcode.util

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign
import kotlin.math.sqrt

class SquidController(
    kP : Double = 0.0,
    kI : Double = 0.0,
    kD : Double = 0.0,
    kF : Double = 0.0,
    maxIntegral: Double = 1.0,
    minIntegral: Double = -1.0
) : PIDController(kP, kI, kD, kF, maxIntegral, minIntegral) {
    override fun calculate(currentPosition : Double, targetPosition : Double): Double {
        // handle time period stuff
        val currentTime = timer.nanoseconds() / 1E9
        if (lastTime == 0.0) lastTime = currentTime
        timePeriod = currentTime - lastTime

        lastTime = currentTime


        // handle error and velocity error
        this.currentPosition = currentPosition
        this.targetPosition = targetPosition

        error = targetPosition - currentPosition
        velError = if (timePeriod != 0.0) {(error - lastError) / timePeriod} else {0.0}
        lastError = error

        // handle integral error
        totalError += timePeriod * error
        totalError = min(maxIntegral, max(minIntegral, totalError))

        // only difference between PIDController.kt and this
        // sqrt of the error to better follow kinematics or whatever
        // https://www.youtube.com/watch?v=WA9o3e4a01Q
        return kP * sqrt(abs(error)) * sign(error) + kI * totalError + kD * velError + targetPosition * kF
    }
}
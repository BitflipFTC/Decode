package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

/**
 * A PID(F) Controller.
 * *Notes*:
 * - feedforward has yet to be tested.
 * - integral term may wind up too quickly.
 *
 * @param kP Proportional control coefficient
 * @param kI Integral control coefficient
 * @param kD Derivative control coefficient
 * @param kF Feedforward coefficient
 * @param maxIntegral Maximum integral value (to prevent windup)
 * @param minIntegral Minimum integral value (to prevent windup)
 */
open class PIDController(
    var kP : Double = 0.0,
    var kI : Double = 0.0,
    var kD : Double = 0.0,
    var kF : Double = 0.0,
    protected var maxIntegral: Double = 1.0,
    protected var minIntegral: Double = -1.0
) {
    var setpoint : Double = 0.0
    var processVariable : Double = 0.0
        protected set
    var setPointTolerance : Double = 1.0
    var velErrorTolerance : Double = Double.POSITIVE_INFINITY

    protected val timer = ElapsedTime()
    protected var lastTime = 0.0
    var timePeriod : Double = 0.0
        protected set
    var lastError : Double = 0.0
        protected set
    var totalError = 0.0
        protected set

    var error = 0.0
        protected set
    var velError = 0.0
        protected set

    init {
        reset()
    }

    fun setCoeffs (kP : Double = 0.0, kI: Double = 0.0, kD: Double = 0.0, kf: Double = 0.0) {
        this.kP = kP
        this.kI = kI
        this.kD = kD
        this.kF = kf
    }

    /**
     * Calculates the output given the current position (process variable) and the target position (setpoint)
     * @param processVariable (process variable) the current position of the system
     * @param setpoint (setpoint) the target position of the system (defaults to the current target)
     * @return the output of the PID Controller
     */
    open fun calculate (processVariable : Double, setpoint : Double = this.setpoint) : Double {
        // handle time period stuff
        val currentTime = timer.nanoseconds() / 1E9
        if (lastTime == 0.0) lastTime = currentTime
        timePeriod = currentTime - lastTime

        lastTime = currentTime


        // handle error and velocity error
        this.processVariable = processVariable
        this.setpoint = setpoint

        error = setpoint - processVariable
        velError = if (timePeriod != 0.0) {(error - lastError) / timePeriod} else {0.0}
        lastError = error

        // handle integral error
        totalError += timePeriod * error
        totalError = min(maxIntegral, max(minIntegral, totalError))

        return kP * error + kI * totalError + kD * velError + setpoint * kF
    }

    /**
     * Resets target, tolerance, time, and error
     */
    fun reset () {
        setpoint = 0.0
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

    /**
     * Checks if the PID Controller is at its set point by checking if the position and velocity errors are within their respective tolerances.
     * @return True if the velocity and position errors are within their tolerances, False otherwise
     */
    fun atSetPoint () : Boolean {
        return (abs(error) <= setPointTolerance) && (abs(velError) <= velErrorTolerance)
    }

    fun resetTotalError () {
        totalError = 0.0
    }
}
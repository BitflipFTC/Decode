package org.firstinspires.ftc.teamcode.subsystems

import org.firstinspires.ftc.teamcode.util.OpModeConstants.telemetry
import org.firstinspires.ftc.teamcode.util.hardware.MotorEx

/**
 * Manages the robot's intake mechanism.
 *
 * This class provides a simple interface for controlling the intake motor.
 * It exposes a `power` property that can be set directly.
 */
class Intake(): Subsystem {
    enum class State (val value: Double) {
        OFF(0.0),
        INTAKE(0.8),
    }

    private var motor: MotorEx = MotorEx("intake").reverse().brake().zeroed()

    var reversed = false

    var debugTelemetry = true

    /**
     * The current power of the intake motor. Can be set to any value between -1.0 and 1.0.
     */
    var power: State = State.OFF

    /**
     * Toggles the intake motor's power between full forward (1.0) and off (0.0).
     */
    fun toggle () {
        power = if (power == State.INTAKE) State.OFF else State.INTAKE
    }

    fun intake () {
        power = State.INTAKE
    }

    fun off () {
        power = State.OFF
    }

    override fun periodic() {
        motor.power = if (reversed) -power.value else power.value

        if (debugTelemetry) {
            telemetry!!.run{
                addData("Intake state", power.name)
                addData("Reversed", reversed)
                addLine("---------------------------")
            }
        }
    }
}

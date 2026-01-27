package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.skeletonarmy.marrow.OpModeManager
import org.firstinspires.ftc.teamcode.util.hardware.MotorEx

/**
 * Manages the robot's intake mechanism.
 *
 * This class provides a simple interface for controlling the intake motor.
 * It exposes a `power` property that can be set directly.
 */
class Intake(): SubsystemBase() {
    enum class State (val value: Double) {
        OFF(0.0),
        INTAKE(1.0 ),
    }

    private val motor: MotorEx = MotorEx("intake").reverse().brake().zeroed()

    var reversed = false

    var debugTelemetry = true

    /**
     * The current power of the intake motor. Can be set to any value between -1.0 and 1.0.
     */
    var power: State = State.OFF

//    /**
//     * Toggles the intake motor's power between full forward (1.0) and off (0.0).
//     */
    fun toggle () {
        power = if (power == State.INTAKE) State.OFF else State.INTAKE
    }

    fun intake () {
        power = State.INTAKE
    }

    fun off () {
        power = State.OFF
    }

//    fun runIntake() = LambdaCommand()
//        .setStart { intake() }
//        .setStop { interrupted -> if (!interrupted) {off()} }
//        .setRequirements(this)
//        .setInterruptible(true)
//        .setName("Intake")
//
//    fun toggleRun() = LambdaCommand()
//        .setStart { toggle() }
//        .setRequirements(this)
//        .setInterruptible(true)
//        .setName("Toggle Intake")
//
//    fun stopIntake() = LambdaCommand()
//        .setStart { off() }
//        .setRequirements(this)
//        .setInterruptible(true)
//        .setName("Stop intake")
//
//    fun reverse() = LambdaCommand()
//        .setStart { this.reversed = true }
//        .setRequirements(this)
//
//    fun forward() = LambdaCommand()
//        .setStart { this.reversed = false }
//        .setRequirements(this)

    override fun periodic() {
        motor.power = if (reversed) -power.value * 1.0 else power.value

        if (debugTelemetry) {
            OpModeManager.getTelemetry()?.run{
                addData("Intake state", power.name)
                addData("Reversed", reversed)
                addLine("---------------------------")
            }
        }
    }
}

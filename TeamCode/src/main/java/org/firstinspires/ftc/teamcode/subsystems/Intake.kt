package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
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
        INTAKE(1.0 ),
    }

    private lateinit var motor: MotorEx

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

    fun runIntake() = LambdaCommand()
//        .setStart { intake() }
//        .setStop { interrupted -> if (!interrupted) {off()} }
//        .setRequirements(this)
//        .setInterruptible(true)
//        .setName("Intake")
//
    fun toggleRun() = LambdaCommand()
//        .setStart { toggle() }
//        .setRequirements(this)
//        .setInterruptible(true)
//        .setName("Toggle Intake")
//
    fun stopIntake() = LambdaCommand()
//        .setStart { off() }
//        .setRequirements(this)
//        .setInterruptible(true)
//        .setName("Stop intake")
//
    fun reverse() = LambdaCommand()
//        .setStart { this.reversed = true }
//        .setRequirements(this)
//
    fun forward() = LambdaCommand()
//        .setStart { this.reversed = false }
//        .setRequirements(this)

    override fun initialize() {
        motor = MotorEx("intake").reverse().brake().zeroed()
    }

    override fun periodic() {
        motor.power = if (reversed) -power.value * 1.0 else power.value

        if (debugTelemetry) {
            ActiveOpMode.telemetry.addData("Intake state", power.name)
            ActiveOpMode.telemetry.addData("Reversed", reversed)
            ActiveOpMode.telemetry.addLine("---------------------------")
        }
    }
}

package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.FunctionalCommand
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.command.StartEndCommand
import com.seattlesolvers.solverslib.command.SubsystemBase
import org.firstinspires.ftc.robotcore.external.Telemetry

/**
 * Manages the robot's intake mechanism.
 *
 * This class provides a simple interface for controlling the intake motor.
 * It exposes a `power` property that can be set directly.
 *
 * @param hwMap The HardwareMap from an OpMode, used to initialize the intake motor.
 */
class Intake(opMode: OpMode): SubsystemBase() {
    enum class State (val value: Double) {
        OFF(0.0),
        INTAKE(0.9),
    }

    val hwMap: HardwareMap = opMode.hardwareMap
    val telemetry: Telemetry = opMode.telemetry

    private val motor by lazy { hwMap["intake"] as DcMotorEx }

    var reversed = false

    /**
     * The current power of the intake motor. Can be set to any value between -1.0 and 1.0.
     */
    var power: State = State.OFF

    fun runIntake()  = RunCommand(this::intake, this)
    fun stopIntake() = RunCommand(this::off, this)
    fun runOuttake() = StartEndCommand({ reversed = true }, { reversed = false }, this)

    init {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor.direction = DcMotorSimple.Direction.REVERSE
        this.defaultCommand = runIntake()
    }

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
        telemetry.addData("Intake state", power.name)
        telemetry.addData("Reversed", reversed)
        telemetry.addLine("---------------------------")
    }
}

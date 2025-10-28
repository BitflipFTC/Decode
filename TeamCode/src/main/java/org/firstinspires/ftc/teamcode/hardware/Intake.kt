package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
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
    val hwMap: HardwareMap = opMode.hardwareMap
    val telemetry: Telemetry = opMode.telemetry

    private val motor by lazy { hwMap["intake"] as DcMotorEx }

    /**
     * The current power of the intake motor. Can be set to any value between -1.0 and 1.0.
     */
    var power: Double
        get() = motor.power
        set(power) { motor.power = power }

    init {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor.direction = DcMotorSimple.Direction.REVERSE
    }

    /**
     * Toggles the intake motor's power between full forward (1.0) and off (0.0).
     */
    fun toggle () {
        power = if (power == 0.0) 1.0 else 0.0
    }
}

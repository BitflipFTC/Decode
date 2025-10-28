package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.config.Config
import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.util.Artifact
import org.firstinspires.ftc.teamcode.util.PIDController

/**
 * Manages the "spindexer" mechanism, a rotating drum with multiple slots for holding artifacts.
 *
 * This class uses a PID controller to move the spindexer to precise angular positions.
 * It tracks the contents of each slot and provides methods for cycling through intake and outtake positions.
 * The [periodic] method must be called in a loop to drive the motor to its target.
 *
 * @param hwMap The HardwareMap from an OpMode, used to initialize the motor.
 */
@Config
@Configurable
class Spindexer(opMode: OpMode): SubsystemBase() {
    companion object {
        const val GEAR_RATIO: Double = 1.375 // 22t out to 16t in
        const val TICKS_PER_REVOLUTION: Double = 537.7 * GEAR_RATIO

        @JvmField
        var kP = 0.001
        @JvmField
        var kI = 0.09
        @JvmField
        var kD = 0.0
    }

    val hwMap: HardwareMap = opMode.hardwareMap
    val telemetry: Telemetry = opMode.telemetry

    /**
     * Defines the named angular positions for the spindexer, used for both intake and outtake.
     * Each position corresponds to a reference angle in degrees.
     */
    enum class Positions(val referenceAngle: Double) {
        INTAKE_ZERO(0.0),
        INTAKE_ONE(120.0),
        INTAKE_TWO(240.0),
        OUTTAKE_ZERO(180.0),
        OUTTAKE_ONE(300.0),
        OUTTAKE_TWO(60.0);
    }

    val positionsToSlotsMap = mapOf(
        Positions.INTAKE_ZERO to 0,
        Positions.OUTTAKE_ZERO to 0,
        Positions.INTAKE_ONE to 1,
        Positions.OUTTAKE_ONE to 1,
        Positions.INTAKE_TWO to 2,
        Positions.OUTTAKE_TWO to 2,
    )

    val slotsToOuttakes = listOf(
        Positions.OUTTAKE_ZERO,
        Positions.OUTTAKE_ONE,
        Positions.OUTTAKE_TWO,
    )

    val slotsToIntakes = listOf(
        Positions.INTAKE_ZERO,
        Positions.INTAKE_ONE,
        Positions.INTAKE_TWO,
    )

    // models physical slots. slots themselves rotate with the spindexer
    private val collectedArtifacts = mutableListOf(
        Artifact.NONE,
        Artifact.NONE,
        Artifact.NONE,
    )

    var ticksTarget: Double = 0.0
        private set

    fun getArtifactString() = collectedArtifacts.joinToString("") { it.firstLetter().toString() }

    private val motor by lazy { hwMap["spindexer"] as DcMotorEx }

    private val controller = PIDController(kP,kI,kD)

    var position = Positions.INTAKE_ZERO
        private set
    var targetAngle = position.referenceAngle
        private set

    init {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.direction = DcMotorSimple.Direction.FORWARD
    }

    fun getAngle() = ( (motor.currentPosition / TICKS_PER_REVOLUTION) * 360 ) % 360

    /**
     * Updates the spindexer's motor power based on the PID controller.
     * This method must be called in a loop for the spindexer to move to its target.
     */
    override fun periodic() {
        controller.setCoeffs(kP,kI,kD)
        val currentPosition: Double = motor.currentPosition.toDouble()

        // creates a "fake" target to ensure the spindexer always takes the shortest path
        ticksTarget = (targetAngle / 360) * TICKS_PER_REVOLUTION
        val pidOutput = controller.calculate(currentPosition, ticksTarget)
        motor.power = pidOutput
    }

    fun setPower(power: Double) {
        motor.power = power
    }

    private fun findFirstFullSlot()   = collectedArtifacts.indexOfFirst { it != Artifact.NONE }
    private fun findFirstEmptySlot()  = collectedArtifacts.indexOf(Artifact.NONE)
    private fun findFirstGreenSlot()  = collectedArtifacts.indexOf(Artifact.GREEN)
    private fun findFirstPurpleSlot() = collectedArtifacts.indexOf(Artifact.PURPLE)

    /**
     * Records that an artifact of a certain color has been collected in the current slot.
     */
    fun recordIntake(color: Artifact) = collectedArtifacts.set(positionsToSlotsMap.getValue(position), color)

    /**
     * Records that an artifact has been removed from the current slot.
     */
    fun recordOuttake() = collectedArtifacts.set(positionsToSlotsMap.getValue(position), Artifact.NONE)

    /**
     * Sets the spindexer's target position.
     * The spindexer will begin moving towards this position on the next [update] call.
     */
    fun setPosition(newPosition: Positions) {
        position = newPosition
        targetAngle = position.referenceAngle
    }

    fun getNamedPosition() = position.name
    fun getPosition() = motor.currentPosition

    val allPositions = Positions.entries.toTypedArray()
    var allPositionsIndex = 0

    /**
     * Cycles to the next position in the [allPositions] list.
     */
    fun toNextPosition() {
        allPositionsIndex = if (allPositionsIndex == allPositions.size - 1) 0 else allPositionsIndex + 1

        setPosition(allPositions[allPositionsIndex])
    }

    var intakePositionsIndex = 0
    /**
     * Cycles to the next intake position.
     */
    fun toNextIntakePosition() {
        intakePositionsIndex = if (intakePositionsIndex == slotsToIntakes.size - 1) 0 else intakePositionsIndex + 1

        setPosition(slotsToIntakes[intakePositionsIndex])
    }

    var outtakePositionsIndex = 0
    /**
     * Cycles to the next outtake position.
     */
    fun toNextOuttakePosition() {
        outtakePositionsIndex = if (outtakePositionsIndex == slotsToOuttakes.size - 1) 0 else outtakePositionsIndex + 1

        setPosition(slotsToOuttakes[outtakePositionsIndex])
    }
}

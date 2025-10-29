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
import kotlin.math.roundToInt

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
class Spindexer(opMode: OpMode) {
    companion object {
        const val GEAR_RATIO: Double = 1.375 // 22t out to 16t in
        const val TICKS_PER_REVOLUTION: Double = 537.7 * GEAR_RATIO

        @JvmField
        var kP = 0.0025
        @JvmField
        var kI = 0.0
        @JvmField
        var kD = 0.0
        @JvmField
        var kS = 0.07
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
    private var targetAngle = position.referenceAngle

    init {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.direction = DcMotorSimple.Direction.FORWARD

        controller.setPointTolerance = 1.toDouble()
    }

    fun resetIntegral() {
        controller.resetTotalError()
    }

    fun getAngle() = ( (motor.currentPosition / TICKS_PER_REVOLUTION) * 360 )
    fun getTargetAngle() = ( (ticksTarget / TICKS_PER_REVOLUTION) * 360)

    /**
     * Updates the spindexer's motor power based on the PID controller.
     * This method must be called in a loop for the spindexer to move to its target.
     */
    fun periodic() {
        controller.setCoeffs(kP, kI, kD, 0.0, kS)
        val currentTicks = motor.currentPosition.toDouble()

        // Convert the target angle (0-360) to a raw, "unwrapped" tick value.
        // TODO: test this logic, it's garbage
        val targetTicksUnwrapped = (targetAngle / 360) * TICKS_PER_REVOLUTION

        // To find the shortest path, we find the equivalent target position that is closest
        // to the motor's current position. We do this by calculating the error in terms of
        // revolutions, rounding to the nearest whole number of revolutions, and then adding
        // that to the unwrapped target to get it onto the same "lap" as the current position.
        val errorInRevolutions = (currentTicks - targetTicksUnwrapped) / TICKS_PER_REVOLUTION
        val nearestRevolution = errorInRevolutions.roundToInt()
        val closestTargetTicks = targetTicksUnwrapped + nearestRevolution * TICKS_PER_REVOLUTION

        // This is the tick value the PID controller will drive towards.
        ticksTarget = closestTargetTicks

        val pidOutput = controller.calculate(currentTicks, ticksTarget)
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

    fun atSetPoint() = controller.atSetPoint()
}

package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.util.Artifact
import org.firstinspires.ftc.teamcode.util.MotifPattern
import org.firstinspires.ftc.teamcode.util.OpModeConstants.telemetry
import org.firstinspires.ftc.teamcode.util.PIDController
import org.firstinspires.ftc.teamcode.util.hardware.MotorEx
import kotlin.math.roundToInt

/**
 * Manages the "spindexer" mechanism, a rotating drum with multiple slots for holding artifacts.
 *
 * This class uses a PID controller to move the spindexer to precise angular positions.
 * It tracks the contents of each slot and provides methods for cycling through intake and outtake positions.
 * The [periodic] method must be called in a loop to drive the motor to its target.
 *
 * It stores artifact positions internally thusly
 *
 *  \      /
 *   \  0 /
 *    \  /
 *     \/
 *  1  |   2
 *     |
 *
 */
@Configurable
class Spindexer(): Subsystem {
    companion object {
        const val GEAR_RATIO: Double = 1.125 // 18t out to 16t in
        const val TICKS_PER_REVOLUTION: Double = 537.7 * GEAR_RATIO

        @JvmField
        var kP = 0.0

        @JvmField
        var kI = 0.0

        @JvmField
        var kD = 0.0

        @JvmField
        var kS = 0.0

        @JvmField
        var turningFeedforward = 0.0

        @JvmField
        var setpointTolerance = 1.0 // in degrees

        @JvmField
        var maxPower = 1.0

        @JvmField
        var tuning = false
    }

    var debugTelemetry = true

    // ------------------ DATA STRUCTURES ------------------
    /**
     * Defines the named angular positions for the spindexer, used for both intake and outtake.
     * Each position corresponds to a reference angle in degrees.
     */
    enum class States(val referenceAngle: Double) {
        INTAKE_ZERO(0.0),
        INTAKE_ONE(120.0),
        INTAKE_TWO(240.0),
        OUTTAKE_ZERO(180.0),
        OUTTAKE_ONE(300.0),
        OUTTAKE_TWO(60.0);
    }

    // specifies the focus slot of each preset (intake / outtake slot)
    val statesToSlotsMap = mapOf(
        States.INTAKE_ZERO to 0,
        States.OUTTAKE_ZERO to 0,
        States.INTAKE_ONE to 1,
        States.OUTTAKE_ONE to 1,
        States.INTAKE_TWO to 2,
        States.OUTTAKE_TWO to 2,
    )

    // allows you to iterate through outtakes or intakes
    // index is the focus slot
    val slotsToOuttakes = listOf(
        States.OUTTAKE_ZERO,
        States.OUTTAKE_ONE,
        States.OUTTAKE_TWO,
    )

    val slotsToIntakes = listOf(
        States.INTAKE_ZERO,
        States.INTAKE_ONE,
        States.INTAKE_TWO,
    )

    val allStates = States.entries.toList()

    // models physical slots. slots themselves rotate with the spindexer
    private val collectedArtifacts: Array<Artifact?> = arrayOf(
        null,
        null,
        null
    )


    // ------------------ EXPOSED FUNCTIONS AND VARIABLES ------------------

    // --------- gettable values ---------

    var state = States.INTAKE_ZERO
        set(newState) {
            // assign it to the backing field. if you assign it to state, infinite loop
            field = newState

            // get current revolution or something
            val errorInRevolutions = (currentAngle - field.referenceAngle) / 360.0
            val nearestRevolution = errorInRevolutions.roundToInt()

            // add that to the existing target to ensure it takes the shortest path
            targetAngle = field.referenceAngle + (nearestRevolution * 360.0)

            // resets atSetPoint() for commands
            controller.calculate(currentTicks, targetTicks)

            resetIntegral()
        }
    val isEmpty: Boolean
        get() = getArtifactString() == "NNN"
    val isFull: Boolean
        get() = !getArtifactString().contains("N")
    val totalFullSlots: Int
        get() = findFullSlots().size
    var targetAngle = state.referenceAngle

    //        private set
    val targetTicks // no backing
        get() = (targetAngle / 360) * TICKS_PER_REVOLUTION
    val currentTicks: Double  // no backing
        get() = motor.currentPosition.toDouble()
    val currentAngle
        get() = (currentTicks / TICKS_PER_REVOLUTION) * 360
    var power: Double
        get() = motor.power
        set(power) {
            motor.power = power
        }
    val hasMotifAssortment: Boolean
        get() = findPurpleSlots().size == 2 && findGreenSlots().size == 1
    var robotTurningPower: Double = 0.0

    var motifPattern: MotifPattern? = null

    fun getArtifactString(): String =
        collectedArtifacts.joinToString("") { it?.firstLetter()?.toString() ?: "N" }

    fun atSetPoint() = controller.atSetPoint()


    // --------- functions that do things ---------

    /** used when setting a new target */
    private fun resetIntegral(): Unit = controller.resetTotalError()

    /**
     * Records that an artifact of a certain color has been collected in the current slot.
     */
    @JvmOverloads
    fun recordIntake(color: Artifact, slot: Int = statesToSlotsMap.getValue(state)) = collectedArtifacts.set(slot, color)

    /**
     * Clears the [collectedArtifacts] variable and replaces it with the arguments.
     */
    fun setCollectedArtifacts(slot0: Artifact, slot1: Artifact, slot2: Artifact) {
        listOf(slot0, slot1, slot2).forEachIndexed { index, artifact -> 
            collectedArtifacts[index] = artifact
        }
    }

    /**
     * Records that an artifact has been removed from the current slot.
     */
    @JvmOverloads
    fun recordOuttake(slot: Int = statesToSlotsMap.getValue(state)) = collectedArtifacts.set(slot, null)

    // --------- functions that control hardware ---------

    private var allStatesIndex = 0

    /**
     * Cycles to the next position in the [allStates] list.
     */
    fun toNextPosition() {
        allStatesIndex = if (allStatesIndex == allStates.size - 1) 0 else allStatesIndex + 1

        state = allStates[allStatesIndex]
    }

    /**
     * Cycles to the next intake position. Turns to 0 if current position is not an intake.
     */
    fun toNextIntakePosition() {
        var targetIndex = 0

        // if current state is an outtake, go to next outtake
        if (slotsToIntakes.indexOf(state) != -1) {
            val currentIndex = slotsToIntakes.indexOf(state)
            targetIndex = if (currentIndex == 2) 0 else currentIndex + 1
        }
        // otherwise, go to OUTTAKE_ZERO

        state = slotsToIntakes[targetIndex]
    }

    fun toFirstEmptyIntakePosition() {
        var targetIndex = 0

        val emptySlots = findEmptySlots()
        if (!emptySlots.isEmpty()) {
            when (emptySlots.size) {
                2    -> {
                    val emptySlot = findEmptySlots().first()

                    // for slots [0,2]
                    // full = 1, so targets 2, then can go 2 -> 0
                    // for [1,2]
                    // full = 1, so targets 1, then can go 1 -> 2
                    // for [0,1]
                    // full = 2, so targets 0, then can go 0 -> 1
                    targetIndex = if (emptySlot == 2) 0 else emptySlot + 1
                }

                1    -> {
                    targetIndex = emptySlots[0]
                }

                // if it's 0 or 3 just go nowhere
                else -> targetIndex = 0
            }
        }
        state = slotsToIntakes[targetIndex]
    }

    /**
     * Cycles to the next outtake position. Turns to 0 if current position is not an outtake.
     */
    fun toNextOuttakePosition() {
        var targetIndex = 0

        // if current state is an outtake, go to next outtake
        if (slotsToOuttakes.indexOf(state) != -1) {
            val currentIndex = slotsToOuttakes.indexOf(state)
            targetIndex = if (currentIndex == 2) 0 else currentIndex + 1
        }
        // otherwise, go to OUTTAKE_ZERO

        state = slotsToOuttakes[targetIndex]
    }

    fun toMotifOuttakePosition() {
        val purpleSlots = findPurpleSlots()
        val greenSlots = findGreenSlots()

        // if we have the correct config of balls
        if (purpleSlots.size == 2 && greenSlots.size == 1) {
            val greenIndex = greenSlots[0]

            // decide which slot should be set to outtake
            // assuming the spindexer will be moving clockwise
            // while shooting
            // If the pattern is GPP, green should be shot first
            // If it's PGP, green should be shot second, etc
            val targetOuttakeIndex = when (motifPattern) {
                MotifPattern.GPP  -> greenIndex
                MotifPattern.PGP  -> if (greenIndex == 0) 2 else greenIndex - 1
                MotifPattern.PPG  -> if (greenIndex == 2) 0 else greenIndex + 1
                null -> 0
            }

            state = slotsToOuttakes[targetOuttakeIndex]
        }
    }

    fun toFirstFullOuttakePosition() {
        val fullSlots = findFullSlots()

        if (!fullSlots.isEmpty()) {
            var targetSlot: Int
            when (fullSlots.size) {
                2    -> {
                    val emptySlot = findEmptySlots().first()

                    // for slots [0,2]
                    // empty = 1, so targets 2, then can go 2 -> 0
                    // for [1,2]
                    // empty = 1, so targets 1, then can go 1 -> 2
                    // for [0,1]
                    // empty = 2, so targets 0, then can go 0 -> 1
                    targetSlot = if (emptySlot == 2) 0 else emptySlot + 1
                }

                1    -> {
                    targetSlot = fullSlots[0]
                }

                else -> {
                    return toNextOuttakePosition()
                }
            }

            state = slotsToOuttakes[targetSlot]
        }
    }


    // ------------------ INTERNAL HARDWARE CONTROL ------------------

    private var motor: MotorEx = MotorEx("spindexer").zeroed().brake()
    private val controller = PIDController(kP, kI, kD, 0.0, kS).apply {
        setPointTolerance = (setpointTolerance / 360) * TICKS_PER_REVOLUTION
    }

    /**
     * Updates the spindexer's motor power based on the PID controller.
     * This method must be called in a loop for the spindexer to move to its target.
     */
    override fun periodic() {
        if (tuning) {
            controller.setCoeffs(kP, kI, kD, 0.0, kS)
        }

        val pidOutput = controller.calculate(currentTicks, targetTicks)
        val motorPower = pidOutput + turningFeedforward * -robotTurningPower
        motor.power = Range.clip(motorPower, -maxPower, maxPower)

        if (debugTelemetry) {
            telemetry!!.run {
                addData("Spindexer target angle", targetAngle)
                addData("Spindexer current angle", currentAngle)
                addData("Spindexer current state", state.name)
                addData("Spindexer atSetPoint", atSetPoint())
                addData("Spindexer indexed artifacts", getArtifactString())
                addData("Spindexer has motif assortment", hasMotifAssortment)
                addData("Motif Pattern", motifPattern?.name ?: "NONE")
                addLine("---------------------------")
            }
        }
    }


    // ------------------ INNER HELPER FUNCTIONS ------------------

    private fun findFullSlots() =
        collectedArtifacts.mapIndexedNotNull { index, artifact -> if (artifact != null) index else null }

    private fun findEmptySlots() =
        collectedArtifacts.mapIndexedNotNull { index, artifact -> if (artifact == null) index else null }

    private fun findGreenSlots() =
        collectedArtifacts.mapIndexedNotNull { index, artifact -> if (artifact == Artifact.GREEN) index else null }

    private fun findPurpleSlots() =
        collectedArtifacts.mapIndexedNotNull { index, artifact -> if (artifact == Artifact.PURPLE) index else null }
}
package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.util.Range
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.util.Artifact
import org.firstinspires.ftc.teamcode.util.commands.IfElseCommand
import org.firstinspires.ftc.teamcode.util.MotifPattern
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
        const val GEAR_RATIO: Double = 1.375 // 22t out to 16t in
        const val TICKS_PER_REVOLUTION: Double = 537.7 * GEAR_RATIO

        @JvmField
        var kP = 0.004

        @JvmField
        var kI = 0.018

        @JvmField
        var kD = 0.00015

        @JvmField
        var kS = 0.015

        @JvmField
        var turningFeedforward = 0.0

        @JvmField
        var setpointTolerance = 1.0 // in degrees

        @JvmField
        var maxPower = 0.5

        @JvmField
        var tuning = false
    }

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
    private val collectedArtifacts = mutableListOf(
        Artifact.NONE,
        Artifact.NONE,
        Artifact.NONE,
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

    var motifPattern: MotifPattern = MotifPattern.NONE

    fun getArtifactString(): String =
        collectedArtifacts.joinToString("") { it.firstLetter().toString() }

    fun atSetPoint() = controller.atSetPoint()


    // --------- functions that do things ---------

    /** used when setting a new target */
    fun resetIntegral(): Unit = controller.resetTotalError()

    /**
     * Records that an artifact of a certain color has been collected in the current slot.
     */
    fun recordIntake(color: Artifact) {
        collectedArtifacts[statesToSlotsMap.getValue(state)] = color
    }

    fun recordIntake(color: Artifact, slot: Int) {
        collectedArtifacts[slot] = color
    }

    /**
     * Records that an artifact has been removed from the current slot.
     */
    fun recordOuttake() = collectedArtifacts.set(statesToSlotsMap.getValue(state), Artifact.NONE)

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
                MotifPattern.NONE -> 0
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

    private lateinit var motor: MotorEx
    private val controller = PIDController(kP, kI, kD, 0.0, kS)

    override fun initialize() {
        motor = MotorEx("spindexer").zeroed().brake()
        // convert from degrees to ticks
        controller.setPointTolerance = (setpointTolerance / 360) * TICKS_PER_REVOLUTION
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

        ActiveOpMode.telemetry.addData("Spindexer target angle", targetAngle)
        ActiveOpMode.telemetry.addData("Spindexer current angle", currentAngle)
        ActiveOpMode.telemetry.addData("Spindexer current state", state.name)
        ActiveOpMode.telemetry.addData("Spindexer atSetPoint", atSetPoint())
        ActiveOpMode.telemetry.addData("Spindexer indexed artifacts", getArtifactString())
        ActiveOpMode.telemetry.addData("Spindexer has motif assortment", hasMotifAssortment)
        ActiveOpMode.telemetry.addData("Motif Pattern", motifPattern.name)
        ActiveOpMode.telemetry.addData("Times to shoot", totalFullSlots)
        ActiveOpMode.telemetry.addLine("---------------------------")
    }


    // ------------------ INNER HELPER FUNCTIONS ------------------

    private fun findFullSlots() =
        collectedArtifacts.mapIndexedNotNull { index, artifact -> if (artifact != Artifact.NONE) index else null }

    private fun findEmptySlots() =
        collectedArtifacts.mapIndexedNotNull { index, artifact -> if (artifact == Artifact.NONE) index else null }

    private fun findGreenSlots() =
        collectedArtifacts.mapIndexedNotNull { index, artifact -> if (artifact == Artifact.GREEN) index else null }

    private fun findPurpleSlots() =
        collectedArtifacts.mapIndexedNotNull { index, artifact -> if (artifact == Artifact.PURPLE) index else null }

    // Commands
    /**
     * Moves to the next intake, or intake 0 if not at an intake.
     */
    fun goToNextIntake() = LambdaCommand()
        .setStart(this::toNextIntakePosition)
        .setIsDone(this::atSetPoint)
        .setRequirements(this)
        .setName("To Next Intake")
        .setInterruptible(true)

    /**
     * Moves to the first empty intake, or intake 0 if all intakes are full or empty.
     */
    fun goToFirstEmptyIntake() = LambdaCommand()
        .setStart(this::toFirstEmptyIntakePosition)
        .setIsDone(this::atSetPoint)
        .setRequirements(this)
        .setName("To First Empty Intake")
        .setInterruptible(true)

    /**
     * Moves to the next outtake, or outtake 0 if not at an outtake.
     */
    fun goToNextOuttake() = LambdaCommand()
        .setStart(this::toNextOuttakePosition)
        .setIsDone(this::atSetPoint)
        .setRequirements(this)
        .setName("To Next Outtake")
        .setInterruptible(true)

    /**
     * Moves to the first full outtake, or outtake 0 if all outtakes are full or empty.
     */
    fun goToFirstFullOuttake() = LambdaCommand()
        .setStart(this::toFirstFullOuttakePosition)
        .setIsDone(this::atSetPoint)
        .setRequirements(this)
        .setName("To Next Outtake")
        .setInterruptible(true)

    /**
     * Moves to an orientation in which the robot can outtake three artifacts in motif order.
     * If the spindexer does not have two purple artifacts and one green artifact, OR there is no motif pattern selected,
     * it will rotate to outtake 0.
     */
    fun goToMotifOuttake() = LambdaCommand()
        .setStart(this::toMotifOuttakePosition)
        .setIsDone(this::atSetPoint)
        .setRequirements(this)
        .setName("To Motif Outtake")
        .setInterruptible(true)

    /**
     * Tries to move to an orientation in which it can outtake artifacts in the motif order. If the spindexer
     * does not have the correct assortment of artifacts, it instead goes to the first full outtake, or outtake 0 if all slots are full or empty.
     */
    fun tryMotifOuttake() = IfElseCommand(
        this::hasMotifAssortment,
        goToMotifOuttake(),
        goToFirstFullOuttake()
    )
}
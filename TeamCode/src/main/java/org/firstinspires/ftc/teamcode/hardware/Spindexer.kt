package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.config.Config
import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range
import com.seattlesolvers.solverslib.command.SubsystemBase
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.util.Artifact
import org.firstinspires.ftc.teamcode.util.MotifPattern
import org.firstinspires.ftc.teamcode.util.PIDController
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
 * @param opMode The OpMode from an OpMode, (this)
 */
@Config
@Configurable
class Spindexer(opMode: OpMode): SubsystemBase() {
    companion object {
        const val GEAR_RATIO: Double = 1.375 // 22t out to 16t in
        const val TICKS_PER_REVOLUTION: Double = 537.7 * GEAR_RATIO

        @JvmField
        var kP = 0.0035

        @JvmField
        var kI = 0.03

        @JvmField
        var kD = 0.0

        @JvmField
        var kS = 0.07

        @JvmField
        var setpointTolerance = 1.0 // in degrees

        @JvmField
        var maxPower = 0.5
    }

    val hwMap: HardwareMap = opMode.hardwareMap
    val telemetry: Telemetry = opMode.telemetry

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
        set (newState) {
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
    val fullSlotNumber: Int
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
                MotifPattern.NONE -> null
            }

            if (targetOuttakeIndex != null) {
                state = slotsToOuttakes[targetOuttakeIndex]
            }
        }
    }


    // ------------------ INTERNAL HARDWARE CONTROL ------------------

    private val motor by lazy { hwMap["spindexer"] as DcMotorEx }
    private val controller = PIDController(kP, kI, kD)

    init {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.direction = DcMotorSimple.Direction.FORWARD

        // convert from degrees to ticks
        controller.setPointTolerance = (setpointTolerance / 360) * TICKS_PER_REVOLUTION
    }

    /**
     * Updates the spindexer's motor power based on the PID controller.
     * This method must be called in a loop for the spindexer to move to its target.
     */
    override fun periodic() {
        controller.setCoeffs(kP, kI, kD, 0.0, kS)

        val pidOutput = controller.calculate(currentTicks, targetTicks)
        motor.power = Range.clip(pidOutput, -maxPower, maxPower)

        telemetry.addData("Spindexer target angle", targetAngle)
        telemetry.addData("Spindexer current angle", currentAngle)
        telemetry.addData("Spindexer current state", state.name)
        telemetry.addData("Spindexer atSetPoint", atSetPoint())
        telemetry.addData("Spindexer indexed artifacts", getArtifactString())
        telemetry.addLine("---------------------------")
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

}

package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.config.Config
import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
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
        @JvmField
        var setpointTolerance = 1.0
    }

    val hwMap: HardwareMap = opMode.hardwareMap
    val telemetry: Telemetry = opMode.telemetry

    // ------------------ DATA STRUCTURES ------------------
    enum class MotifPattern {
        GPP,
        PGP,
        PPG,
        NONE
    }

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
        private set
    var targetAngle = state.referenceAngle
        private set
    val targetTicks // no backing
        get() = (targetAngle / 360) * TICKS_PER_REVOLUTION
    val currentTicks: Double  // no backing
        get() = motor.currentPosition.toDouble()
    val currentAngle
        get() = (currentTicks / TICKS_PER_REVOLUTION) * 360
    var power: Double
        get() = motor.power
        set(power) { motor.power = power }

    var motifPattern: MotifPattern = MotifPattern.NONE

    fun getArtifactString(): String = collectedArtifacts.joinToString("") { it.firstLetter().toString() }
    fun atSetPoint() = controller.atSetPoint()


    // --------- functions that do things ---------

    /** used when setting a new target */
    fun resetIntegral(): Unit = controller.resetTotalError()

    /**
     * Records that an artifact of a certain color has been collected in the current slot.
     */
    fun recordIntake(color: Artifact) = collectedArtifacts.set(statesToSlotsMap.getValue(state), color)

    /**
     * Records that an artifact has been removed from the current slot.
     */
    fun recordOuttake() = collectedArtifacts.set(statesToSlotsMap.getValue(state), Artifact.NONE)

    /**
     * Sets the spindexer's target position.
     * The spindexer will begin moving towards this position on the next [periodic] call.
     */
    fun setTargetState(newState: States) {
        state = newState

        // find the current "revolution"
        val errorInRevolutions = (currentAngle - state.referenceAngle) / 360.0
        val nearestRevolution = errorInRevolutions.roundToInt()

        // add that to the existing target to ensure it takes the shortest path
        targetAngle = state.referenceAngle + (nearestRevolution * 360.0)

        resetIntegral()
    }


    // --------- functions that control hardware ---------

    private var allStatesIndex = 0
    private var intakePositionsIndex = 0
    private var outtakePositionsIndex = 0

    /**
     * Cycles to the next position in the [allStates] list.
     */
    fun toNextPosition() {
        allStatesIndex = if (allStatesIndex == allStates.size - 1) 0 else allStatesIndex + 1

        setTargetState(allStates[allStatesIndex])
    }

    /**
     * Cycles to the next intake position.
     */
    fun toNextIntakePosition() {
        intakePositionsIndex = if (intakePositionsIndex == slotsToIntakes.size - 1) 0 else intakePositionsIndex + 1

        setTargetState(slotsToIntakes[intakePositionsIndex])
    }

    /**
     * Cycles to the next outtake position.
     */
    fun toNextOuttakePosition() {
        outtakePositionsIndex = if (outtakePositionsIndex == slotsToOuttakes.size - 1) 0 else outtakePositionsIndex + 1

        setTargetState(slotsToOuttakes[outtakePositionsIndex])
    }


    // ------------------ INTERNAL HARDWARE CONTROL ------------------

    private val motor by lazy { hwMap["spindexer"] as DcMotorEx }
    private val controller = PIDController(kP, kI, kD)

    init {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.direction = DcMotorSimple.Direction.FORWARD

        controller.setPointTolerance = setpointTolerance
    }

    /**
     * Updates the spindexer's motor power based on the PID controller.
     * This method must be called in a loop for the spindexer to move to its target.
     */
    fun periodic() {
        controller.setCoeffs(kP, kI, kD, 0.0, kS)

        val pidOutput = controller.calculate(currentAngle, targetAngle)
        motor.power = pidOutput
    }


    // ------------------ INNER HELPER FUNCTIONS ------------------

    private fun findFirstFullSlot()   = collectedArtifacts.indexOfFirst { it != Artifact.NONE }
    private fun findFirstEmptySlot()  = collectedArtifacts.indexOf(Artifact.NONE)
    private fun findFirstGreenSlot()  = collectedArtifacts.indexOf(Artifact.GREEN)
    private fun findFirstPurpleSlot() = collectedArtifacts.indexOf(Artifact.PURPLE)
}

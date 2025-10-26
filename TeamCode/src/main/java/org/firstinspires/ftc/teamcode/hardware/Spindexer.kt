package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.config.Config
import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.util.Artifact
import org.firstinspires.ftc.teamcode.util.PIDController
import kotlin.math.PI

@Config
@Configurable
class Spindexer(val hwMap: HardwareMap) {
    companion object {
        const val GEAR_RATIO: Double = 1.375
        const val TICKS_PER_REVOLUTION: Double = 145.1 * GEAR_RATIO

        @JvmField
        var kP = 0.0

        @JvmField
        var kI = 0.0

        @JvmField
        var kD = 0.0
    }

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

    fun getArtifactString() = collectedArtifacts.joinToString("") { it.firstLetter().toString() }

    private val motor by lazy { hwMap["spindexer"] as DcMotorEx }

    private val controller = PIDController(kP,kI,kD)

    private var position = Positions.INTAKE_ZERO
    private var targetAngle = position.referenceAngle

    init {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.direction = DcMotorSimple.Direction.FORWARD
    }

    fun getAngle() = ( (motor.currentPosition / TICKS_PER_REVOLUTION) * 360 ) % 360

    fun update(): List<Double> {
        controller.setCoeffs(kP,kI,kD)
        val currentAngle = getAngle()
        // creates a "fake" target to ensure the spindexer always takes the shortest path
        val localTarget = (targetAngle - currentAngle + 540) % 360 - 180
        val pidOutput = controller.calculate(currentAngle, localTarget)
        motor.power = pidOutput
        return listOf(currentAngle, targetAngle)
    }

    private fun findFirstFullSlot()   = collectedArtifacts.indexOfFirst { it != Artifact.NONE }
    private fun findFirstEmptySlot()  = collectedArtifacts.indexOf(Artifact.NONE)
    private fun findFirstGreenSlot()  = collectedArtifacts.indexOf(Artifact.GREEN)
    private fun findFirstPurpleSlot() = collectedArtifacts.indexOf(Artifact.PURPLE)

    fun recordIntake(color: Artifact) = collectedArtifacts.set(positionsToSlotsMap.getValue(position), color)
    fun recordOuttake() = collectedArtifacts.set(positionsToSlotsMap.getValue(position), Artifact.NONE)

    fun setPosition(newPosition: Positions) {
        position = newPosition
        targetAngle = position.referenceAngle
    }

    fun getPosition() = position.name

    val allPositions = Positions.entries.toTypedArray()
    var allPositionsIndex = 0

    fun toNextPosition() {
        allPositionsIndex = if (allPositionsIndex == allPositions.size - 1) 0 else allPositionsIndex + 1

        setPosition(allPositions[allPositionsIndex])
    }
}

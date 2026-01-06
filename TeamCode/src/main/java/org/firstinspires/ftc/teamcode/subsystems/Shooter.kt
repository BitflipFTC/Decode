package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.VoltageSensor
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.util.InterpolatedLookupTable
import org.firstinspires.ftc.teamcode.util.PIDController
import org.firstinspires.ftc.teamcode.util.hardware.MotorEx
import org.firstinspires.ftc.teamcode.util.hardware.ServoEx

/**
 * Manages the robot's shooter mechanism, controlling a flywheel and an adjustable hood servo.
 *
 * This class encapsulates the hardware and control logic for the shooter. To use it,
 * you must instantiate it with the `hardwareMap`, and then call the [periodic] method
 * repeatedly in your OpMode's main loop.
 *
 * The two primary control parameters are [targetFlywheelRPM] and [hoodPosition].
 * Setting these properties will cause the [periodic] method to adjust the physical
 * hardware to match the targets.
 *
 * The flywheel speed is managed by a PID controller, and its constants ([kP], [kI], [kD], [kV])
 * can be tuned.
 */
@Configurable
class Shooter(): Subsystem {
    companion object {
        const val FLYWHEEL_PPR = 28
        const val LOW_PASS = 0.05

        @JvmField
        var kP = 0.05
        @JvmField
        var kV = 0.0026

        @JvmField
        var tuning = false
    }

    class ShooterState (
        val angle: Double,
        val rpm: Double,
    )

    private lateinit var vSensor: VoltageSensor
    private lateinit var hoodServo: ServoEx
    private lateinit var flywheelMotor: MotorEx
    private val flywheelController = PIDController(kP, 0.0, 0.0, kV)

    // long goal is approximately 125 in. from peak of long to center of goal tag
    // longest short zone is approx. 80 in. from peak to center
    // shortest we can see from is about 25.0.
    val distanceArray = doubleArrayOf(
        41.0,
        60.0,
        75.0,
        84.0,
        130.0
    )

    val speedArray = doubleArrayOf(
        2500.0,
        2999.0,
        3000.0,
        3150.0,
        4000.0
    )

    val angleArray = doubleArrayOf(
        0.0,
        0.05,
        0.3,
        0.6,
        0.6,
    )

    private val velocityLookupTable = InterpolatedLookupTable(
        distanceArray,
        speedArray
    )

    private val angleLookupTable = InterpolatedLookupTable(
        speedArray,
        angleArray
    )

    // main two adjustable params

    var targetFlywheelRPM = 0.0
    var hoodPosition = 0.0

    var flywheelRPM = 0.0
        private set
    var lastFlywheelRPM = 0.0
        private set
    var filteredFlywheelRPM = 0.0
        private set
    var pidOutput = 0.0
        private set
    var distance = 0.0

    var debugTelemetry = true

    override fun initialize() {
        flywheelMotor = MotorEx("flywheel").zeroed().float().reverse().apply {
            maxSlewRate = 0.2
        }

        hoodServo = ServoEx("hood").apply{
            position = hoodPosition
        }

        flywheelController.setPointTolerance = 65.toDouble()

        vSensor = ActiveOpMode.hardwareMap.get(VoltageSensor::class.java, "Control Hub")
    }

    private var cachedVoltage = 13.0
    private var voltageTimer = 0

    override fun periodic() {
        if (voltageTimer++ % 50 == 0) {
            cachedVoltage = vSensor.voltage
        }

        lastFlywheelRPM = flywheelRPM
        flywheelRPM = (flywheelMotor.velocity / FLYWHEEL_PPR) * 60
        filteredFlywheelRPM = flywheelRPM * LOW_PASS + lastFlywheelRPM * (1 - LOW_PASS)

        if (tuning) {
            flywheelController.setCoeffs(kP, 0.0, 0.0, kV, 0.0)
        }

        pidOutput = flywheelController.calculate(filteredFlywheelRPM, targetFlywheelRPM)
        
        // allow it to stop SLOWLY when target is 0
        flywheelMotor.power = if (flywheelController.error <= -500) 0.0 else pidOutput / cachedVoltage

        hoodServo.position = hoodPosition

        if (debugTelemetry) {
            ActiveOpMode.telemetry.addData("Flywheel target RPM", targetFlywheelRPM)
            ActiveOpMode.telemetry.addData("Flywheel current RPM", flywheelRPM)
            ActiveOpMode.telemetry.addData("Flywheel at set point", atSetPoint())
            ActiveOpMode.telemetry.addData("Hood position", hoodPosition)
            ActiveOpMode.telemetry.addLine("---------------------------")
        }
    }

    fun atSetPoint() = flywheelController.atSetPoint()

    fun setTargetState(distance: Double) {
        // ensure the parameter distance is actually based on an apriltag reading
        if (distance > 0.0) {
            targetFlywheelRPM = velocityLookupTable.calculate(distance)
            hoodPosition = angleLookupTable.calculate(targetFlywheelRPM)
        }

        this.distance = distance
    }
}

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
 * The flywheel speed is managed by a PID controller, and its constants ([kP], [kV])
 * can be tuned.
 */
@Configurable
class Shooter(): Subsystem {
    companion object {
        const val GEAR_RATIO = 58/60 // output / input
        const val FLYWHEEL_PPR = 28 * GEAR_RATIO
        const val LOW_PASS = 0.1

        @JvmField
        var kP = 0.03
        @JvmField
        var kV = 0.00244

        @JvmField
        var tuning = false

        const val LP_FILTER = 0.01
    }

    private lateinit var vSensor: VoltageSensor
    private lateinit var hoodServo: ServoEx
    private lateinit var flywheelMotor: MotorEx
    private val flywheelController = PIDController(kP, 0.0, 0.0, kV)

    // long goal is approximately 125 in. from peak of long to center of goal tag
    // longest short zone is approx. 80 in. from peak to center
    // shortest we can see from is about 25.0.
    val distanceArray = doubleArrayOf(
        39.0,
        52.0,
        58.0,
        67.0,
        102.0,
        137.0,
        152.0
    )
    val speedArray = doubleArrayOf(
        2750.0,
        2800.0,
        2850.0,
        3000.0,
        3677.0,
        4005.0,
        4175.0
    )
    val angleArray = doubleArrayOf(
        0.45,
        0.475,
        0.5,
        0.55,
        0.55,
        0.6,
        0.6
    )
    val timeInAirArray = doubleArrayOf(
        0.5,
        0.57,
        0.64,
        0.67,
        0.83,
        0.9,
        0.9
    )

    private val velocityLookupTable = InterpolatedLookupTable(
        distanceArray,
        speedArray
    )

    private val angleLookupTable = InterpolatedLookupTable(
        distanceArray,
        angleArray
    )

    private val timeInAirLookupTable = InterpolatedLookupTable(
        distanceArray,
        timeInAirArray
    )

    // main two adjustable params

    var targetFlywheelRPM = 0.0
    var hoodPosition = 0.0

    var expectedTimeInAir = 0.0
        private set

    var flywheelRPM = 0.0
        private set
    var filteredFlywheelRPM = 0.0
        private set
    var pidOutput = 0.0
        private set
    var distance = 0.0

//    private var compensatingForShot = false

//    fun compensateForShot() {
//        compensatingForShot = true
//    }

    var debugTelemetry = true

    override fun initialize() {
        flywheelMotor = MotorEx("flywheel").zeroed().floa().reverse().apply {
            maxSlewRate = 0.5
        }

        hoodServo = ServoEx("hood").apply{
            position = hoodPosition
        }

        flywheelController.setPointTolerance = 35.0

        vSensor = ActiveOpMode.hardwareMap.get(VoltageSensor::class.java, "Control Hub")
    }

    private var cachedVoltage: Double = 13.0
    private var voltageTimer = 0

    override fun periodic() {
        if (voltageTimer++ % 50 == 0) {
            cachedVoltage = LP_FILTER * vSensor.voltage + (1 - LP_FILTER) * cachedVoltage
        }

        flywheelRPM = (flywheelMotor.velocity / FLYWHEEL_PPR) * 60
        filteredFlywheelRPM = flywheelRPM * LOW_PASS + filteredFlywheelRPM * (1 - LOW_PASS)

        if (tuning) {
            flywheelController.setCoeffs(kP, 0.0, 0.0, kV, 0.0)
        }

//        if (filteredFlywheelRPM >= targetFlywheelRPM && compensatingForShot) {
//            compensatingForShot = false
//        }

//        if (!compensatingForShot) {
            pidOutput = flywheelController.calculate(filteredFlywheelRPM, targetFlywheelRPM)

            // allow it to stop SLOWLY when target is 0
            flywheelMotor.power = pidOutput / cachedVoltage
//        } else {
//            flywheelMotor.power = 1.0
//        }

        hoodServo.position = hoodPosition

        if (debugTelemetry) {
            ActiveOpMode.telemetry.addData("Flywheel target RPM", targetFlywheelRPM)
            ActiveOpMode.telemetry.addData("Flywheel current RPM", filteredFlywheelRPM)
            ActiveOpMode.telemetry.addData("Flywheel at set point", atSetPoint())
            ActiveOpMode.telemetry.addData("Voltage", "%06.2fV", cachedVoltage)
            ActiveOpMode.telemetry.addData("Hood position", hoodPosition)
            ActiveOpMode.telemetry.addLine("---------------------------")
        }
    }

    fun atSetPoint() = flywheelController.atSetPoint()

    fun setTargetState(distance: Double) {
        // ensure the parameter distance is actually based on an apriltag reading
        if (distance > 0.0) {
            targetFlywheelRPM = velocityLookupTable.calculate(distance)
            hoodPosition = angleLookupTable.calculate(distance)
            expectedTimeInAir = timeInAirLookupTable.calculate(distance)
        }

        this.distance = distance
    }
}

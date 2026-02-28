package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.utils.MovingAverageSmoother
import com.qualcomm.robotcore.hardware.VoltageSensor
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.util.InterpolatedLookupTable
import org.firstinspires.ftc.teamcode.util.PIDController
import org.firstinspires.ftc.teamcode.util.hardware.MotorEx
import org.firstinspires.ftc.teamcode.util.hardware.ServoEx
import kotlin.math.abs

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
        const val GEAR_RATIO = 60.0/58.0 // output / input
        const val FLYWHEEL_PPR = 28 * GEAR_RATIO
        const val LOW_PASS = 0.1

        @JvmField
        var kP = 0.015
        @JvmField
        var kV = 0.00248

        @JvmField
        var useVelocityCorrection = true

        @JvmField
        var tuning = false

        const val VOLTAGE_FILTER = 0.01

        // 125 rpm change is approximately fixed by a 2.5 degree lowering of the hood
        // we have an angle range of exactly 35 degrees to 60 degrees.
        const val HOOD_GEAR_RATIO = 167/21
        // now assuming hood 0.0 = 35 deg,
        // 0.8 is going to be 65 deg (dw abt the math)
        // 37.7245508981 total range in degrees
        // 0.53 is 55 deg
    }

    fun hoodPosToDegrees(pos: Double) = (30.0 / 0.8) * pos + 35
    fun degreesToHoodPos(degrees: Double) = (degrees - 35) / (30.0 / 0.8)

    private lateinit var vSensor: VoltageSensor
    private lateinit var hoodServo: ServoEx
    private lateinit var flywheelMotor: MotorEx
    private val flywheelController = PIDController(kP, 0.0, 0.0, kV)

    // long goal is approximately 125 in. from peak of long to center of goal tag
    // longest short zone is approx. 80 in. from peak to center
    // shortest we can see from is about 25.0.
    val distanceArray = doubleArrayOf(
        46.0,
        72.0,
        97.0,
        137.0,
        152.0,
    )
    val speedArray = doubleArrayOf(
        3125.0,
        3625.0,
        3875.0,
        4500.0,
        4750.0,
    )
    val angleArray = doubleArrayOf(
        0.025,
        0.35,
        0.3875,
        0.4,
        0.425
    )
    val timeInAirArray = doubleArrayOf(
        0.5,
        0.55,
        0.65,
        0.8,
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

    val velocityFilter = MovingAverageSmoother(3)

    var debugTelemetry = true

    override fun initialize() {
        flywheelMotor = MotorEx("flywheel").zeroed().floa().reverse().apply {
            maxSlewRate = 0.5
        }

        hoodServo = ServoEx("hood").apply{
            position = hoodPosition
        }

        flywheelController.setPointTolerance = 50.0

        vSensor = ActiveOpMode.hardwareMap.get(VoltageSensor::class.java, "Control Hub")
    }

//    private var cachedVoltage: Double = 13.0
//    private var voltageTimer = 0

    override fun periodic() {
//        if (voltageTimer++ % 50 == 0) {
//            cachedVoltage = VOLTAGE_FILTER * vSensor.voltage + (1 - VOLTAGE_FILTER) * cachedVoltage
//        }

        flywheelRPM = (flywheelMotor.velocity / FLYWHEEL_PPR) * 60
        filteredFlywheelRPM = velocityFilter.addValue(flywheelRPM)

        if (tuning) {
            flywheelController.setCoeffs(kP, 0.0, 0.0, kV, 0.0)
        }

//        pidOutput = flywheelController.calculate(filteredFlywheelRPM, targetFlywheelRPM)

//        flywheelMotor.power = pidOutput / cachedVoltage

        flywheelMotor.power = if (filteredFlywheelRPM < targetFlywheelRPM && abs(targetFlywheelRPM) >= 250.0) {
            1.0
        } else if (targetFlywheelRPM - filteredFlywheelRPM <= -750.0 && abs(targetFlywheelRPM) >= 250.0) {
            -0.25
        } else 0.0


        hoodServo.position = hoodPosition

        if (debugTelemetry) {
            ActiveOpMode.telemetry.addData("Flywheel target RPM", targetFlywheelRPM)
            ActiveOpMode.telemetry.addData("Flywheel current RPM", filteredFlywheelRPM)
            ActiveOpMode.telemetry.addData("Flywheel at set point", atSetPoint())
//            ActiveOpMode.telemetry.addData("Voltage", "%06.2fV", cachedVoltage)
            ActiveOpMode.telemetry.addData("Hood position", hoodPosition)
            ActiveOpMode.telemetry.addLine("---------------------------")
        }
    }

    fun atSetPoint() = abs(targetFlywheelRPM - filteredFlywheelRPM) <= 100.0

    fun setTargetState(distance: Double) {
        // ensure the parameter distance is actually based on an apriltag reading
        if (distance > 0.0) {
            targetFlywheelRPM = velocityLookupTable.calculate(distance)
            setTimeInAir(distance)

            val firstHoodPosition = angleLookupTable.calculate(distance)

            if (useVelocityCorrection) {
                val firstHoodDegrees = hoodPosToDegrees(firstHoodPosition)
                val flywheelDiff = targetFlywheelRPM - filteredFlywheelRPM
                val hoodOffsetVeloCorrection = ((-1.0 * (distance / 150.0)) * flywheelDiff) / 125.0
                val targetHoodPosition =
                    degreesToHoodPos(firstHoodDegrees + hoodOffsetVeloCorrection)
                hoodPosition = targetHoodPosition.coerceIn(0.025..0.45) // limit from 35 to 55 deg
            } else {
                hoodPosition = firstHoodPosition
            }
        }

        this.distance = distance
    }

    fun setTimeInAir(distance: Double) {
        expectedTimeInAir = timeInAirLookupTable.calculate(distance)
    }
}

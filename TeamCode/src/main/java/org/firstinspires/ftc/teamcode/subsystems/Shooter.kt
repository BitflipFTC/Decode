package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.skeletonarmy.marrow.OpModeManager
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
class Shooter(): SubsystemBase() {
    companion object {
        const val FLYWHEEL_PPR = 28
        const val LOW_PASS = 0.05

        @JvmField
        var kP = 0.01
        @JvmField
        var kV = 0.00245

        @JvmField
        var tuning = false
    }


    // long goal is approximately 125 in. from peak of long to center of goal tag
    // longest short zone is approx. 80 in. from peak to center
    // shortest we can see from is about 25.0.
    val distanceArray = doubleArrayOf(
        41.0,
        60.0,
        82.0,
        104.0,
        139.0,
        150.0
    )

    val speedArray = doubleArrayOf(
        2750.0,
        2875.0,
        3150.0,
        3500.0,
        3875.0,
        4000.0
    )

    val angleArray = doubleArrayOf(
        0.01,
        0.175,
        0.3,
        0.35,
        0.6,
        0.6
    )

    private val velocityLookupTable = InterpolatedLookupTable(
        distanceArray,
        speedArray
    )

    private val angleLookupTable = InterpolatedLookupTable(
        distanceArray,
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

    private var vSensor: VoltageSensor =
        OpModeManager.getHardwareMap().get(VoltageSensor::class.java, "Control Hub")
    private var hoodServo: ServoEx = ServoEx("hood").apply{
        position = hoodPosition
    }
    private var flywheelMotor: MotorEx = MotorEx("flywheel").zeroed().float().reverse().apply {
        maxSlewRate = 0.2
    }
    private val flywheelController = PIDController(kP, 0.0, 0.0, kV).apply {
        setPointTolerance = 35.0
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
        flywheelMotor.power = pidOutput / cachedVoltage

        hoodServo.position = hoodPosition

        if (debugTelemetry) {
            OpModeManager.getTelemetry()?.run{
                addData("Flywheel target RPM", targetFlywheelRPM)
                addData("Flywheel current RPM", flywheelRPM)
                addData("Flywheel at set point", atSetPoint())
                addData("Hood position", hoodPosition)
                addLine("---------------------------")
            }
        }
    }

    fun atSetPoint() = flywheelController.atSetPoint()

    fun setTargetState(distance: Double) {
        // ensure the parameter distance is actually based on an apriltag reading
        if (distance > 0.0) {
            targetFlywheelRPM = velocityLookupTable.calculate(distance)
            hoodPosition = angleLookupTable.calculate(distance)
        }

        this.distance = distance
    }
}

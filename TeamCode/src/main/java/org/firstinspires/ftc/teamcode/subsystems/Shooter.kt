package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.util.InterpolatedLookupTable
import org.firstinspires.ftc.teamcode.util.OpModeConstants.hardwareMap
import org.firstinspires.ftc.teamcode.util.OpModeConstants.telemetry
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
        var kP = 0.009
        @JvmField
        var kV = 0.0024

        @JvmField
        var tuning = false
    }

    class ShooterState (
        val angle: Double,
        val rpm: Double,
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
    var state = ShooterState(targetFlywheelRPM,hoodPosition)
        set(state) {
            field = state
            targetFlywheelRPM = state.rpm
            hoodPosition = state.angle
        }
    var distance = 0.0

    var debugTelemetry = true

    private var vSensor: VoltageSensor = hardwareMap!!.get(VoltageSensor::class.java, "Control Hub")
    private var hoodServo: ServoEx = ServoEx("hood").apply{
        position = hoodPosition
    }
    private var flywheelMotor: MotorEx = MotorEx("flywheel").zeroed().float().reverse().apply {
        maxSlewRate = 0.2
    }
    private val flywheelController = PIDController(kP, 0.0, 0.0, kV).apply {
        setPointTolerance = 85.0
    }

    // long goal is approximately 125 in. from peak of long to center of goal tag
    // longest short zone is approx. 80 in. from peak to center
    // shortest we can see from is about 25.0.
    private val lookupTable = InterpolatedLookupTable(
        doubleArrayOf(
        ),
        doubleArrayOf(
        ),
        doubleArrayOf(
        )
    )

    override fun periodic() {
        lastFlywheelRPM = flywheelRPM
        flywheelRPM = (flywheelMotor.velocity / FLYWHEEL_PPR) * 60
        filteredFlywheelRPM = flywheelRPM * LOW_PASS + lastFlywheelRPM * (1 - LOW_PASS)

        if (tuning) {
            flywheelController.setCoeffs(kP, 0.0, 0.0, kV, 0.0)
        }

        pidOutput = flywheelController.calculate(filteredFlywheelRPM, targetFlywheelRPM)
        
        // allow it to stop SLOWLY when target is 0
        flywheelMotor.power = if (flywheelController.error <= -500) 0.0 else pidOutput / vSensor.voltage

        hoodServo.position = hoodPosition

        if (debugTelemetry) {
            telemetry!!.addData("Flywheel target RPM", targetFlywheelRPM)
            telemetry!!.addData("Flywheel current RPM", flywheelRPM)
            telemetry!!.addData("Flywheel at set point", atSetPoint())
            telemetry!!.addData("Hood position", hoodPosition)
            telemetry!!.addLine("---------------------------")
        }
    }

    fun atSetPoint() = flywheelController.atSetPoint()

    fun setTargetState(distance: Double) {
        // ensure the parameter distance is actually based on an apriltag reading
        if (distance > 0.0) {
            state = lookupTable.calculate(distance)
        }

        this.distance = distance
    }
}

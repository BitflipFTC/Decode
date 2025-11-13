package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.subsystems.Shooter.Companion.kD
import org.firstinspires.ftc.teamcode.subsystems.Shooter.Companion.kI
import org.firstinspires.ftc.teamcode.subsystems.Shooter.Companion.kP
import org.firstinspires.ftc.teamcode.subsystems.Shooter.Companion.kV
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
@Config
class Shooter(): Subsystem {
    companion object {
        const val FLYWHEEL_PPR = 28
        const val LOW_PASS = 0.05

        const val PEAK_NEAR_LAUNCH_ZONE = 83.0
        const val CLOSE_SHOOTING = 32.1
        
        @JvmField
        var kP = 0.005
        @JvmField
        var kI = 0.0
        @JvmField
        var kD = 0.0
        @JvmField
        var kV = 0.00255

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
    private val flywheelController = PIDController(kP, kI, kD, kV)

    // todo tune at 35.0, 50.0, 65.0, 80.0, 100.0, 120.0, 140.0.
    // long goal is approximately 125 in. from peak of long to center of goal tag
    // longest short zone is approx. 80 in. from peak to center
    // shortest we can see from is about 35.0.
    private val lookupTable = InterpolatedLookupTable(
        doubleArrayOf(
            35.0,
            50.0,
            65.0,
            80.0,
            100.0,
            120.0,
            140.0
        ),
        doubleArrayOf(
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        ),
        doubleArrayOf(
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        )
    )

    // main two adjustable params

    var targetFlywheelRPM = 0.0
    var hoodPosition = 0.3

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

    override fun initialize() {
        flywheelMotor = MotorEx("flywheel").zeroed().float().reverse().apply {
            maxSlewRate = 0.2
        }

        hoodServo = ServoEx("hood")

        hoodServo.position = hoodPosition
        flywheelController.setPointTolerance = 155.toDouble()

        vSensor = ActiveOpMode.hardwareMap.get(VoltageSensor::class.java, "Control Hub")
    }

    override fun periodic() {
        lastFlywheelRPM = flywheelRPM
        flywheelRPM = (flywheelMotor.velocity / FLYWHEEL_PPR) * 60
        filteredFlywheelRPM = flywheelRPM * LOW_PASS + lastFlywheelRPM * (1 - LOW_PASS)

        if (tuning) {
            flywheelController.setCoeffs(kP, kI, kD, kV, 0.0)
        }

        pidOutput = flywheelController.calculate(filteredFlywheelRPM, targetFlywheelRPM)
        
        // allow it to stop SLOWLY when target is 0
        flywheelMotor.power = if (flywheelController.error <= -500) 0.0 else pidOutput / vSensor.voltage

        hoodServo.position = hoodPosition

        ActiveOpMode.telemetry.addData("Flywheel target RPM", targetFlywheelRPM)
        ActiveOpMode.telemetry.addData("Flywheel current RPM", flywheelRPM)
        ActiveOpMode.telemetry.addData("Flywheel at set point", atSetPoint())
        ActiveOpMode.telemetry.addData("Hood position", hoodPosition)
        ActiveOpMode.telemetry.addLine("---------------------------")
    }

    fun atSetPoint() = flywheelController.atSetPoint()

    fun calculateTargetState(distance: Double) {
        // ensure the parameter distance is actually based on an apriltag reading
        if (distance > 0.0) {
            state = lookupTable.calculate(distance)
        }

        this.distance = distance
    }
}

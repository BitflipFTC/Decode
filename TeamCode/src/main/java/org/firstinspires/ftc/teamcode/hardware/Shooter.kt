package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.seattlesolvers.solverslib.command.SubsystemBase
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.hardware.Shooter.Companion.kD
import org.firstinspires.ftc.teamcode.hardware.Shooter.Companion.kI
import org.firstinspires.ftc.teamcode.hardware.Shooter.Companion.kP
import org.firstinspires.ftc.teamcode.hardware.Shooter.Companion.kV
import org.firstinspires.ftc.teamcode.util.InterpolatedLookupTable
import org.firstinspires.ftc.teamcode.util.PIDController

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
 * can be tuned via the FTC-Dashboard. The hood servo's limits ([SERVO_LOWER_LIMIT],
 * [SERVO_UPPER_LIMIT]) are also tunable.
 *
 * @param hwMap The HardwareMap from your OpMode, used to initialize the motors and servos.
 */
@Config
class Shooter(opMode: OpMode): SubsystemBase() {
    companion object {
        const val FLYWHEEL_PPR = 28
        const val LOW_PASS = 0.05

        const val PEAK_NEAR_LAUNCH_ZONE = 83.0
        const val CLOSE_SHOOTING = 32.1
        
        @JvmField
        var kP = 0.003
        @JvmField
        var kI = 0.0
        @JvmField
        var kD = 0.0
        @JvmField
        var kV = 0.00234

        @JvmField
        var tuning = false
    }

    class ShooterState (
        val angle: Double,
        val rpm: Double,
//        val estimatedShotTime: Double
    )

    val hwMap: HardwareMap = opMode.hardwareMap
    val telemetry: Telemetry = opMode.telemetry

    private var vSensor: VoltageSensor = hwMap.get(VoltageSensor::class.java, "Control Hub")
    private val hoodServo by lazy { hwMap["hood"] as ServoImplEx }
    private val flywheelMotor by lazy { hwMap["flywheel"] as DcMotorEx }
    private val flywheelController = PIDController(kP, kI, kD, kV)
    private val lookupTable = InterpolatedLookupTable(
        doubleArrayOf(
            32.0,
            45.0,
            60.0,
            84.0,
            115.0,
            ),
        doubleArrayOf(
            0.5,
            0.5,
            0.3,
            0.25,
            0.1,
            ),
        doubleArrayOf(
            3100.0,
            3350.0,
            3600.0,
            4250.0,
            4650.0,
        )
    )

    // main two adjustable params

    //todo uncomment
    var targetFlywheelRPM = 3000.0
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

    init {
        hoodServo.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
//        hoodServo.scaleRange(SERVO_LOWER_LIMIT, SERVO_UPPER_LIMIT)
        hoodServo.position = hoodPosition

        flywheelMotor.direction = DcMotorSimple.Direction.REVERSE
        flywheelMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        flywheelMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        flywheelMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT

        flywheelController.setPointTolerance = 65.toDouble()
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

        // hood stuff
//        hoodServo.scaleRange(SERVO_LOWER_LIMIT, SERVO_UPPER_LIMIT)
        hoodServo.position = hoodPosition

        telemetry.addData("Flywheel target RPM", targetFlywheelRPM)
        telemetry.addData("Flywheel current RPM", flywheelRPM)
        telemetry.addData("Flywheel at set point", atSetPoint())
        telemetry.addData("Hood position", hoodPosition)
        telemetry.addData("Distance to goal", distance)
        telemetry.addLine("---------------------------")
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

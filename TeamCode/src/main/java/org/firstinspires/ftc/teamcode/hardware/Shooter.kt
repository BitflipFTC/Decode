package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.util.PIDController

/**
 * Manages the robot's shooter mechanism, controlling a flywheel and an adjustable hood servo.
 *
 * This class encapsulates the hardware and control logic for the shooter. To use it,
 * you must instantiate it with the `hardwareMap`, and then call the [update] method
 * repeatedly in your OpMode's main loop.
 *
 * The two primary control parameters are [targetFlywheelRPM] and [hoodPosition].
 * Setting these properties will cause the [update] method to adjust the physical
 * hardware to match the targets.
 *
 * The flywheel speed is managed by a PID controller, and its constants ([kP], [kI], [kD], [kV])
 * can be tuned via the FTC-Dashboard. The hood servo's limits ([SERVO_LOWER_LIMIT],
 * [SERVO_UPPER_LIMIT]) are also tunable.
 *
 * @param hwMap The HardwareMap from your OpMode, used to initialize the motors and servos.
 */
@Config
class Shooter(val hwMap: HardwareMap) {
    companion object {
        const val FLYWHEEL_PPR = 28
        const val LOW_PASS = 0.05
        
        @JvmField
        var SERVO_LOWER_LIMIT = 0.0
        @JvmField
        var SERVO_UPPER_LIMIT = 1.0
        @JvmField
        var kP = 0.0007
        @JvmField
        var kI = 0.0
        @JvmField
        var kD = 0.0
        @JvmField
        var kV = 0.00018
    }
    private val hoodServo by lazy { hwMap["hood"] as ServoImplEx }
    private val flywheelMotor by lazy { hwMap["flywheel"] as DcMotorEx }
    private val flywheelController = PIDController(kP, kI, kD, kV)

    var flywheelRPM = 0.0
        private set
    var lastFlywheelRPM = 0.0
        private set
    var filteredFlywheelRPM = 0.0
        private set
    var pidOutput = 0.0
        private set
    
    // main two adjustable params
    var targetFlywheelRPM = 3000.0
    var hoodPosition = 0.3

    init {
        hoodServo.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        hoodServo.scaleRange(SERVO_LOWER_LIMIT, SERVO_UPPER_LIMIT)
        hoodServo.position = hoodPosition

        flywheelMotor.direction = DcMotorSimple.Direction.REVERSE
        flywheelMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        flywheelMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        flywheelMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT

        flywheelController.setPointTolerance = 85.toDouble()
    }
    
    fun update() {
        lastFlywheelRPM = flywheelRPM
        flywheelRPM = (flywheelMotor.velocity / FLYWHEEL_PPR) * 60
        filteredFlywheelRPM = flywheelRPM * LOW_PASS + lastFlywheelRPM * (1 - LOW_PASS)
        
        pidOutput = flywheelController.calculate(filteredFlywheelRPM, targetFlywheelRPM)
        
        // allow it to stop SLOWLY when target is 0
        flywheelMotor.power = if (flywheelController.error <= -750) 0.0 else pidOutput

        // hood stuff
        hoodServo.scaleRange(SERVO_LOWER_LIMIT, SERVO_UPPER_LIMIT)
        hoodServo.position = hoodPosition
    }
}

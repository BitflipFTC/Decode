package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.PwmControl
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.util.PIDController

@Config
class Turret(): Subsystem {
    companion object {
        @JvmField
        var kP = 0.004
        @JvmField
        var kD = 0.0012
        @JvmField
        var kS = 0.0272
        @JvmField
        var kV = -0.22
        @JvmField
        var setPointTolerance : Double = 5.toDouble() // degrees
        @JvmField
        var tuning = false
    }

    private lateinit var servoL: CRServoImplEx
    private lateinit var servoR: CRServoImplEx
    private val controller = PIDController(kP, 0.0, kD, 0.0, kS)

    var bearing = 0.0
    var turningPower = 0.0
    var pidOutput: Double = 0.0
        private set

    override fun initialize() {
        servoL = ActiveOpMode.hardwareMap.get(CRServoImplEx::class.java, "turretL").apply{
            pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        }

        servoR = ActiveOpMode.hardwareMap.get(CRServoImplEx::class.java, "turretR").apply {
            pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        }

        controller.setPointTolerance = setPointTolerance
    }

    fun setPower (pow : Double) {
        servoR.power = -pow
        servoL.power = -pow
    }

    override fun periodic() {
        pidOutput = controller.calculate(bearing, 0.0) // bearing approaches 0

        setPower(pidOutput + kV * turningPower)
1
        if (tuning) {
            controller.setCoeffs(kP, 0.0, kD, 0.0, kS)
        }

        ActiveOpMode.telemetry.addData("Turret at set point", atSetPoint())
        ActiveOpMode.telemetry.addData("Turret bearing", bearing)
        ActiveOpMode.telemetry.addData("Turret target bearing", 0.0)
        ActiveOpMode.telemetry.addData("Turret power", getPower())

        ActiveOpMode.telemetry.addLine("---------------------------")
    }

    fun getPower() = servoR.power

    fun atSetPoint() = controller.atSetPoint()
}
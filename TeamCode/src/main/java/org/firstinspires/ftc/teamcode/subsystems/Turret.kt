package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.util.PIDController
import org.firstinspires.ftc.teamcode.util.hardware.CRServoEx

@Configurable
object Turret: Subsystem {
    @JvmField
    var kP = 0.0031
    @JvmField
    var kD = 0.00055
    @JvmField
    var kS = 0.04
    @JvmField
    var kV = -0.2
    @JvmField
    var setPointTolerance : Double = 3.toDouble() // degrees
    @JvmField
    var tuning = false

    private lateinit var servoL: CRServoEx
    private lateinit var servoR: CRServoEx
    private val controller = PIDController(kP, 0.0, kD, 0.0, kS)

    var bearing = 0.0
    var turningPower = 0.0
    var pidOutput: Double = 0.0
        private set

    override fun initialize() {
        servoL = CRServoEx("turretL")
        servoR = CRServoEx("turretR")

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
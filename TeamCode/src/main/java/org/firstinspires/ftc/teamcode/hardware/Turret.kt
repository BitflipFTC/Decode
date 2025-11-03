package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.seattlesolvers.solverslib.command.SubsystemBase
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.util.PIDController

@Config
class Turret(opMode: OpMode): SubsystemBase() {
    companion object {
        @JvmField
        var kP = 0.006
        @JvmField
        var kD = 0.0012
        @JvmField
        var kS = 0.0272
        @JvmField
        var setPointTolerance : Double = 5.toDouble() // degrees
    }

    val hwMap: HardwareMap = opMode.hardwareMap
    val telemetry: Telemetry = opMode.telemetry

    val servoL : CRServoImplEx by lazy { hwMap["turretL"] as CRServoImplEx }
    val servoR : CRServoImplEx by lazy { hwMap["turretR"] as CRServoImplEx }
    val controller = PIDController(kP, 0.0, kD, 0.0, kS)


    var bearing = 0.0
    var pidOutput: Double = 0.0

    init {
        // defaults to 600,2400 - this gives full range
        servoR.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        servoL.pwmRange = PwmControl.PwmRange(500.0, 2500.0)

        controller.setPointTolerance = setPointTolerance
    }

    fun setPower (pow : Double) {
        servoR.power = -pow
        servoL.power = -pow
    }

    override fun periodic() {
        pidOutput = controller.calculate(bearing, 0.0) // bearing approaches 0

        setPower(pidOutput)

        controller.setCoeffs(kP, 0.0, kD, 0.0, kS)

        telemetry.addData("Turret at set point", atSetPoint())
    }

    fun getPower() = servoR.power

    fun atSetPoint() = controller.atSetPoint()
}
package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.seattlesolvers.solverslib.command.SubsystemBase
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.util.PIDController
import org.firstinspires.ftc.teamcode.util.TurretTestPID
import org.firstinspires.ftc.teamcode.util.TurretTestPID.targetTagPos
import kotlin.math.sign

@Config
class Turret(opMode: OpMode) {
    companion object {
        @JvmField
        var kP = 0.0003
        @JvmField
        var kI = 0.0
        @JvmField
        var kD = 0.0
        @JvmField
        var kS = 0.0275
        @JvmField
        var setPointTolerance : Double = 25.toDouble()
    }

    val hwMap: HardwareMap = opMode.hardwareMap
    val telemetry: Telemetry = opMode.telemetry

    val servoL : CRServoImplEx by lazy { hwMap["turretL"] as CRServoImplEx }
    val servoR : CRServoImplEx by lazy { hwMap["turretR"] as CRServoImplEx }
    val controller = PIDController(kP, kI, kD, 0.0, kS)

    var pidOutput: Double = 0.0
    init {
        // defaults to 600,2400 - this gives full range
        servoR.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        servoL.pwmRange = PwmControl.PwmRange(500.0, 2500.0)

        controller.setPointTolerance = setPointTolerance
    }

    fun setPower (pow : Double) {
        servoR.power = pow
        servoL.power = pow
    }

    fun periodic(tagPos: Double) {
        pidOutput = controller.calculate(tagPos, 320.0)

        if (!controller.atSetPoint()) {
            // idk why kS is added again
            // but it worked in turret test so uhhhh leave it
            // trust
            // signed,
            // 10/28/25
            pidOutput += sign(pidOutput) * TurretTestPID.kS
        }

        setPower(pidOutput)
    }

    fun getPower() = pidOutput

    fun atSetPoint() = controller.atSetPoint()
}
package org.firstinspires.ftc.teamcode.subsystems

import android.annotation.SuppressLint
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.util.hardware.MotorEx
import kotlin.math.max

class Drivetrain(val hardwareMap: HardwareMap): Subsystem {
    class DrivePowers(val fl: Double, val fr: Double, val bl: Double, val br: Double) {
        @SuppressLint("DefaultLocale")
        override fun toString(): String {
            return String.format("FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f", fl, fr, bl, br)
        }

        fun normalized(): DrivePowers {
            val max = max(
                max(fl, fr),
                max(bl,br)
            )

            return if (max > 1.00) {
                DrivePowers(
                    fl / max, fr / max, bl / max, br / max
                )
            } else {
                DrivePowers(
                    fl, fr, bl, br
                )
            }
        }
    }

    var currentDrivePowers = DrivePowers(0.0,0.0,0.0,0.0)
        private set
    var heading = 0.0
        private set

    var debugTelemetry = true

    var fieldCentric = false
    var driveSpeed = 1.0

    private lateinit var frontLeft : MotorEx
    private lateinit var frontRight: MotorEx
    private lateinit var backLeft  : MotorEx
    private lateinit var backRight : MotorEx

    override fun initialize() {
        frontLeft = MotorEx("frontleft").reverse().zeroed().brake()
        frontRight = MotorEx("frontright").reverse().brake().zeroed()
        backLeft = MotorEx("backleft").zeroed().brake()
        backRight = MotorEx("backright").reverse().zeroed().brake()
    }

    fun setDrivetrainPowers(powers: DrivePowers) {
        frontLeft.power = powers.fl
        frontRight.power = powers.fr
        backLeft.power = powers.bl
        backRight.power = powers.br

        currentDrivePowers = powers
    }

    fun calculateDrivetrainPowers (strafe: Double, forward: Double, yaw: Double): DrivePowers {
        return DrivePowers(
            (forward + strafe + yaw) * driveSpeed,
            (forward - strafe - yaw) * driveSpeed,
            (forward - strafe + yaw) * driveSpeed,
            (forward + strafe - yaw) * driveSpeed
        ).normalized()
    }

    fun resetYaw () {
    }

    override fun periodic() {
        if (debugTelemetry) {
            ActiveOpMode.telemetry.addLine(currentDrivePowers.toString())
            ActiveOpMode.telemetry.addData("Heading", heading)
        }
    }
}
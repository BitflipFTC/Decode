package org.firstinspires.ftc.teamcode.subsystems

import android.annotation.SuppressLint
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.util.hardware.MotorEx
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

class Drivetrain(): Subsystem {
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

    var fieldCentric = false
    var driveSpeed = 1.0

    private lateinit var imu: IMU
    private lateinit var frontLeft : MotorEx
    private lateinit var frontRight: MotorEx
    private lateinit var backLeft  : MotorEx
    private lateinit var backRight : MotorEx

    override fun initialize() {
        frontLeft = MotorEx("frontleft").reverse().zeroed().brake()
        frontRight = MotorEx("frontright").brake().zeroed()
        backLeft = MotorEx("backleft").reverse().zeroed().brake()
        backRight = MotorEx("backright").zeroed().brake()

        imu = ActiveOpMode.hardwareMap.get(IMU::class.java, "imu").apply {
            initialize(
                IMU.Parameters(
                    RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                    )
                )
            )
            resetYaw()
        }

        heading = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
    }

    fun setDrivetrainPowers(powers: DrivePowers) {
        frontLeft.power = powers.fl
        frontRight.power = powers.fr
        backLeft.power = powers.bl
        backRight.power = powers.br

        currentDrivePowers = powers
    }

    fun calculateDrivetrainPowers (strafe: Double, forward: Double, yaw: Double): DrivePowers {

        if (!fieldCentric) {
            return DrivePowers(
                (forward + strafe + yaw) * driveSpeed,
                (forward - strafe - yaw) * driveSpeed,
                (forward - strafe + yaw) * driveSpeed,
                (forward + strafe - yaw) * driveSpeed
            ).normalized()
        } else {
            heading = -imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
            val rotStrafe = strafe * cos(heading) - forward * sin(heading)
            val rotForward = strafe * sin(heading) + forward * cos(heading)

            return DrivePowers(
                (rotForward + rotStrafe + yaw) * driveSpeed,
                (rotForward - rotStrafe - yaw) * driveSpeed,
                (rotForward - rotStrafe + yaw) * driveSpeed,
                (rotForward + rotStrafe - yaw) * driveSpeed
            ).normalized()
        }
    }

    fun resetYaw () {
        imu.resetYaw()
    }

    override fun periodic() {
        ActiveOpMode.telemetry.addLine(currentDrivePowers.toString())
        ActiveOpMode.telemetry.addData("Heading", heading)
    }
}
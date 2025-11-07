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
    var driveSpeed = 0.8

    private lateinit var imu: IMU
    private lateinit var frontLeft : DcMotorEx
    private lateinit var frontRight: DcMotorEx
    private lateinit var backLeft  : DcMotorEx
    private lateinit var backRight : DcMotorEx

    override fun initialize() {
        frontLeft = ActiveOpMode.hardwareMap.get(DcMotorEx::class.java, "frontleft").apply {
            direction = DcMotorSimple.Direction.REVERSE
            mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        frontRight = ActiveOpMode.hardwareMap.get(DcMotorEx::class.java, "frontright").apply {
            mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        backLeft = ActiveOpMode.hardwareMap.get(DcMotorEx::class.java, "backleft").apply {
            direction = DcMotorSimple.Direction.REVERSE
            mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        backRight = ActiveOpMode.hardwareMap.get(DcMotorEx::class.java, "backright").apply {
            mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

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
        heading = imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES)

        if (!fieldCentric) {
            return DrivePowers(
                (forward + strafe + yaw) * driveSpeed,
                (forward - strafe - yaw) * driveSpeed,
                (forward - strafe + yaw) * driveSpeed,
                (forward + strafe - yaw) * driveSpeed
            ).normalized()
        } else {
            val rotStrafe = strafe * cos(Math.toRadians(-heading)) - forward * sin(Math.toRadians(-heading))
            val rotForward = strafe * sin(Math.toRadians(-heading)) + forward * cos(Math.toRadians(-heading))

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
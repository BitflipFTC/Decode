package org.firstinspires.ftc.teamcode.hardware

import android.annotation.SuppressLint
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.seattlesolvers.solverslib.command.SubsystemBase
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

class Drivetrain(opMode: OpMode): SubsystemBase() {
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

    val hwMap: HardwareMap = opMode.hardwareMap
    val telemetry: Telemetry = opMode.telemetry

    var currentDrivePowers = DrivePowers(0.0,0.0,0.0,0.0)
        private set
    var heading = 0.0
        private set

    var fieldCentric = false
    var driveSpeed = 0.8

    private val imu by lazy { hwMap["imu"] as IMU }
    private val frontLeft  by lazy { hwMap["frontleft"]  as DcMotorEx }
    private val frontRight by lazy { hwMap["frontright"] as DcMotorEx }
    private val backLeft   by lazy { hwMap["backleft"]   as DcMotorEx }
    private val backRight  by lazy { hwMap["backright"]  as DcMotorEx }

    init {
        frontLeft.direction = DcMotorSimple.Direction.REVERSE
        backLeft.direction  = DcMotorSimple.Direction.REVERSE

        frontLeft.mode  = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        frontRight.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        backLeft.mode   = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        backRight.mode  = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        frontLeft.zeroPowerBehavior  = DcMotor.ZeroPowerBehavior.BRAKE
        frontRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        backLeft.zeroPowerBehavior   = DcMotor.ZeroPowerBehavior.BRAKE
        backRight.zeroPowerBehavior  = DcMotor.ZeroPowerBehavior.BRAKE

        frontLeft.mode  = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        frontRight.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        backLeft.mode   = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        backRight.mode  = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        imu.initialize(IMU.Parameters(RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)))
        imu.resetYaw()
        heading = imu.robotYawPitchRollAngles.yaw
    }

    fun setDrivetrainPowers(powers: DrivePowers) {
        frontLeft.power = powers.fl
        frontRight.power = powers.fr
        backLeft.power = powers.bl
        backRight.power = powers.br

        currentDrivePowers = powers
    }

    fun calculateDrivetrainPowers (strafe: Double, forward: Double, yaw: Double): DrivePowers {
        heading = imu.robotYawPitchRollAngles.yaw

        if (!fieldCentric) {
            return DrivePowers(
                (forward + strafe + yaw.toDouble()) * driveSpeed,
                (forward - strafe - yaw.toDouble()) * driveSpeed,
                (forward - strafe + yaw.toDouble()) * driveSpeed,
                (forward + strafe - yaw.toDouble()) * driveSpeed
            ).normalized()
        } else {
            val rotStrafe = strafe * cos(Math.toRadians(-heading)) - forward * sin(Math.toRadians(-heading))
            val rotForward = strafe * sin(Math.toRadians(-heading)) + forward * sin(Math.toRadians(-heading))

            return DrivePowers(
                (rotForward + rotStrafe + yaw.toDouble()) * driveSpeed,
                (rotForward - rotStrafe - yaw.toDouble()) * driveSpeed,
                (rotForward - rotStrafe + yaw.toDouble()) * driveSpeed,
                (rotForward + rotStrafe - yaw.toDouble()) * driveSpeed
            ).normalized()
        }
    }

    fun resetYaw () {
        imu.resetYaw()
    }
}
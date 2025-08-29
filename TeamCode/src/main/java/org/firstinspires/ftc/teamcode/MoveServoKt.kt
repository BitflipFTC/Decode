package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime

@TeleOp(name = "servo0kt")
class MoveServoKt : LinearOpMode() {
    private lateinit var servo: Servo
    private lateinit var servo2: Servo

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        servo = hardwareMap.get(Servo::class.java, "servo")
        servo2 = hardwareMap.get(Servo::class.java, "servo2")
        var position = 0.0

        val timer = ElapsedTime()
        val gp1 = Gamepad()

        var diff : Float
        waitForStart()
        timer.reset()
        while (opModeIsActive()) {
            diff = gp1.left_stick_x;
            telemetry.addData("time",timer.milliseconds())
            position += diff * 0.005
            servo.position = position
            servo2.position = position
        }
    }
}
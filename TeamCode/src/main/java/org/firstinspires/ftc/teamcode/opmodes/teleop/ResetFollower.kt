package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class ResetFollower: LinearOpMode() {
    override fun runOpMode() {
        CombinedTeleOp.follower = null
    }
}
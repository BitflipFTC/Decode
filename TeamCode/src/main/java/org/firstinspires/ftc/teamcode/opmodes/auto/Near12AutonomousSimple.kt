package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.opmodes.auto.paths.Near12
import org.firstinspires.ftc.teamcode.opmodes.teleop.CombinedTeleOp.Companion.follower
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.FiniteStateMachine
import org.firstinspires.ftc.teamcode.util.FollowPathState
import org.firstinspires.ftc.teamcode.util.InitializeState
import org.firstinspires.ftc.teamcode.util.InstantState
import org.firstinspires.ftc.teamcode.util.TimedFollowPathState
import org.firstinspires.ftc.teamcode.util.WaitState
import org.firstinspires.ftc.teamcode.util.WaitUntilState

@Suppress("Unused")
@Autonomous(name = "NEAR 12 Ball", preselectTeleOp = "Combined TeleOp")
class Near12AutonomousSimple: BaseAutonomous() {
    override fun initialize(alliance: Alliance) {
        pathSequence = Near12(alliance)
        paths = pathSequence.buildPaths(follower!!)
        val timer = ElapsedTime()

        follower!!.setStartingPose(pathSequence.poses.nearStartPose)

        finiteStateMachine = FiniteStateMachine(
            FollowPathState("Score preload", paths[0]),
            InstantState("aaaa",{turret.automatic = true}),
            WaitState(750.0),
            WaitUntilState(shooter::atSetPoint),
            shootState(),

            FollowPathState("dintake 2", paths[1]),
            WaitState(125.0),
            FollowPathState("DScore 2", paths[2]),
            shootState(),

            FollowPathState("Do gate drive", paths[3]),
            TimedFollowPathState("Do gate empty", paths[4], 750.0),
            WaitState(250.0),
            TimedFollowPathState("Do gate intake", paths[5], 2000.0),
            InstantState("") {timer.reset()},
            WaitUntilState { spindexer.isFull || timer.milliseconds() >= 2000.0 },
            FollowPathState("Score gate intake", paths[6]),
            shootState(),

            FollowPathState("intake 1", paths[7]),
            WaitState(125.0),
            FollowPathState("dscore1", paths[8]),
            shootState(),
        )
    }


}
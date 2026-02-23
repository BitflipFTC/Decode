package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.opmodes.auto.paths.Near15
import org.firstinspires.ftc.teamcode.opmodes.teleop.CombinedTeleOp.Companion.follower
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.FiniteStateMachine
import org.firstinspires.ftc.teamcode.util.FollowPathState
import org.firstinspires.ftc.teamcode.util.InstantState
import org.firstinspires.ftc.teamcode.util.State
import org.firstinspires.ftc.teamcode.util.WaitState
import org.firstinspires.ftc.teamcode.util.WaitUntilState

@Suppress("Unused")
@Autonomous(name = "NEAR 15 Ball", preselectTeleOp = "Combined TeleOp")
class Near15AutonomousSimple: BaseAutonomous() {
    override fun initialize(alliance: Alliance) {
        pathSequence = Near15(alliance)
        paths = pathSequence.buildPaths(follower!!)
        follower!!.setStartingPose(pathSequence.poses.nearStartPose)
        val timer = ElapsedTime()

        finiteStateMachine = FiniteStateMachine(
            FollowPathState("Score preload", paths[0]),
            startIntake(),
            shootState(),

            FollowPathState("dintake 2", paths[1]),
            FollowPathState("intake 2", paths[2]),
            WaitState(150.0),
            FollowPathState("DScore 3", paths[3]),
            shootState(),

            FollowPathState("drive to gate intake", paths[4]),
            FollowPathState("Do gate empty", paths[5]),
            WaitState(250.0),
            FollowPathState("Do gate intake", paths[6]),
            InstantState("") {timer.reset()},
            WaitUntilState { spindexer.isFull || timer.milliseconds() >= 2500.0 },
            FollowPathState("Score gate intake", paths[7]),

            FollowPathState("dintake 1", paths[8]),
            FollowPathState("intake 1", paths[9]),
            WaitState(150.0),
            FollowPathState("dscore1", paths[10]),
            shootState(),

            FollowPathState("dIntake 3", paths[11]),
            FollowPathState("intake 3", paths[12]),
            WaitState(150.0),
            FollowPathState("DScore 3", paths[13]),
            shootState(),

            FollowPathState("Park", paths[14])
        )
    }
}
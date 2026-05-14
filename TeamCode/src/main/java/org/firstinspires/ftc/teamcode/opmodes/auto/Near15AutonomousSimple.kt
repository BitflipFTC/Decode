package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.opmodes.auto.paths.Near15
import org.firstinspires.ftc.teamcode.opmodes.teleop.CombinedTeleOp.Companion.follower
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.FiniteStateMachine
import org.firstinspires.ftc.teamcode.util.FollowPathState
import org.firstinspires.ftc.teamcode.util.InstantState
import org.firstinspires.ftc.teamcode.util.TimedFollowPathState
import org.firstinspires.ftc.teamcode.util.TurnToState
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
            WaitUntilState(shooter::atSetPoint),
            shootState(),

            FollowPathState("dintake 2", paths[1]),
            FollowPathState("DScore 2", paths[2]),
            TurnToState(paths[2].path.endPose().heading, 1000.0),
            shootState(),

            TimedFollowPathState("Do gate drive", paths[3], 1000.0),
            TimedFollowPathState("Do gate empty", paths[4], 750.0),
//            WaitState(250.0),
            TimedFollowPathState("Do gate intake", paths[5], 1000.0),
            InstantState("") {timer.reset()},
            WaitUntilState { spindexer.isFull || timer.milliseconds() >= 2000.0 },
            TimedFollowPathState("Score gate intake", paths[6], 2000.0),
            TurnToState(paths[6].path.endPose().heading, 1000.0),
            shootState(),

            TimedFollowPathState("Do gate drive", paths[7], 1000.0),
            TimedFollowPathState("Do gate empty", paths[8], 750.0),
//            WaitState(250.0),
            TimedFollowPathState("Do gate intake", paths[9], 1000.0),
            InstantState("") {timer.reset()},
            WaitUntilState { spindexer.isFull || timer.milliseconds() >= 2000.0 },
            TimedFollowPathState("Score gate intake", paths[10], 2000.0),
            TurnToState(paths[10].path.endPose().heading, 1000.0),
            shootState(),

            TimedFollowPathState("intake 1", paths[11], 2000.0),
            WaitState(500.0),
            TimedFollowPathState("dscore1", paths[12],2000.0),

            shootState(),

        )
    }
}
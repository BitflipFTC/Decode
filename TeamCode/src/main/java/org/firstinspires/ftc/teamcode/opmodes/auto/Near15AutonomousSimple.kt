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
import org.firstinspires.ftc.teamcode.util.TimedFollowPathState
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
            InstantState("zeep", { turret.automatic = false
                turret.angle = 90.0} ),
            FollowPathState("Score preload", paths[0]),
            startIntake(),
//            WaitUntilState( { spindexer.motifPattern != null }),
            InstantState("aaaa",{turret.automatic = true}), // per the code the turret goes to auto
            // when the motif is detected. this is a fallback in case the motif isn't detected.
            WaitState(350.0),
            WaitUntilState(shooter::atSetPoint),
            shootState(),

            FollowPathState("dintake 2", paths[1]),
//            WaitState(375.0),
            FollowPathState("DScore 2", paths[2]),
            shootState(),

            FollowPathState("Do gate drive", paths[3]),
            TimedFollowPathState("Do gate empty", paths[4], 750.0),
//            WaitState(250.0),
            FollowPathState("Do gate intake", paths[5]),
            InstantState("") {timer.reset()},
            WaitUntilState { spindexer.isFull || timer.milliseconds() >= 2000.0 },
//            stopIntake(),
            FollowPathState("Score gate intake", paths[6]),
//            startIntake(),
            shootState(),

            FollowPathState("intake 1", paths[7]),
//            WaitState(500.0),
            FollowPathState("dscore1", paths[8]),
            shootState(),

            FollowPathState("intake 3", paths[9]),
//            WaitState(150.0),
//            stopIntake(),
            FollowPathState("DScore 3", paths[10]),
//            startIntake(),
            shootState(),
        )
    }
}
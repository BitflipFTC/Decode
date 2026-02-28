package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.auto.paths.Near12
import org.firstinspires.ftc.teamcode.opmodes.teleop.CombinedTeleOp.Companion.follower
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.FiniteStateMachine
import org.firstinspires.ftc.teamcode.util.FollowPathState
import org.firstinspires.ftc.teamcode.util.TimedFollowPathState
import org.firstinspires.ftc.teamcode.util.WaitState

@Suppress("Unused")
@Autonomous(name = "NEAR 12 Ball", preselectTeleOp = "Combined TeleOp")
class Near12AutonomousSimple: BaseAutonomous() {
    override fun initialize(alliance: Alliance) {
        pathSequence = Near12(alliance)
        paths = pathSequence.buildPaths(follower!!)
        follower!!.setStartingPose(pathSequence.poses.nearStartPose)

        finiteStateMachine = FiniteStateMachine(
            FollowPathState("Score preload", paths[0]),
            startIntake(),
            shootState(),

            FollowPathState("dintake 1", paths[1]),
            FollowPathState("intake 1", paths[2]),
            WaitState(150.0),
            FollowPathState("dramping it", paths[3]),
            TimedFollowPathState("ramping ", paths[4], 2000.0),
            WaitState(100.0),
            FollowPathState("dscore1", paths[5]),
            shootState(),

            FollowPathState("dintake 2", paths[6]),
            FollowPathState("intake 2", paths[7]),
            WaitState(150.0),
            FollowPathState("DScore 3", paths[8]),
            shootState(),

            FollowPathState("dIntake 3", paths[9]),
            FollowPathState("intake 3", paths[10]),
            WaitState(150.0),
            FollowPathState("DScore 3", paths[11]),
            shootState(),

            FollowPathState("Park", paths[12])
        )
    }
}
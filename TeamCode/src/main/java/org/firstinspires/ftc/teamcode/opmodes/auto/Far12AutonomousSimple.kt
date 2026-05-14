package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.auto.paths.Far12
import org.firstinspires.ftc.teamcode.opmodes.teleop.CombinedTeleOp.Companion.follower
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.FiniteStateMachine
import org.firstinspires.ftc.teamcode.util.TimedFollowPathState
import org.firstinspires.ftc.teamcode.util.WaitState
import org.firstinspires.ftc.teamcode.util.WaitUntilState

@Suppress("Unused")
@Autonomous(name = "FAR 12 Ball", preselectTeleOp = "Combined TeleOp")
class Far12AutonomousSimple: BaseAutonomous() {
    override fun initialize(alliance: Alliance) {
        pathSequence = Far12(alliance)
        paths = pathSequence.buildPaths(follower!!)
        follower!!.setStartingPose(pathSequence.poses.farStartPose)

        finiteStateMachine = FiniteStateMachine(
            TimedFollowPathState("Score preload", paths[0], 1000.0),
            startIntake(),
            WaitUntilState(shooter::atSetPoint),
            shootState(),

            TimedFollowPathState("D 3", paths[1], 4000.0),
            WaitState(150.0),
            TimedFollowPathState("D 3", paths[2], 2000.0),
            shootState(),

            TimedFollowPathState("D HP", paths[3], 3000.0),
            WaitState(150.0),
            TimedFollowPathState("D Score", paths[4], 2000.0),
            shootState(),

            TimedFollowPathState("dtointake", paths[5], 3000.0),
            TimedFollowPathState("d to shoito", paths[6], 2000.0),
            shootState(),

            TimedFollowPathState("Park", paths[7], 2000.0)
        )
    }
}
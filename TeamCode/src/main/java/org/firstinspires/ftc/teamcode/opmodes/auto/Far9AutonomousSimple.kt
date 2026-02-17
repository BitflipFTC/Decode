package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.auto.paths.Far9
import org.firstinspires.ftc.teamcode.opmodes.teleop.CombinedTeleOp.Companion.follower
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.FiniteStateMachine
import org.firstinspires.ftc.teamcode.util.FollowPathState
import org.firstinspires.ftc.teamcode.util.WaitState

@Suppress("Unused")
@Autonomous(name = "FAR 9 Ball", preselectTeleOp = "Combined TeleOp")
class Far9AutonomousSimple: BaseAutonomous() {
    override fun initialize(alliance: Alliance) {
        pathSequence = Far9(alliance)
        paths = pathSequence.buildPaths(follower!!)

        finiteStateMachine = FiniteStateMachine(
            FollowPathState("D Score Preload", paths[0]),
            startIntake(),
            WaitState(250.0),
            getMotifState(),
            shootState(),
            FollowPathState("D Intake 1", paths[1]),
            WaitState(150.0),
            FollowPathState("D Score 1", paths[2]),
            shootState(),
            FollowPathState("D Intake 2", paths[3]),
            WaitState(150.0),
            FollowPathState("D Score 2", paths[4]),
            shootState(),
            FollowPathState("Park", paths[5])
        )
    }
}
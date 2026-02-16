package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.auto.paths.Near12
import org.firstinspires.ftc.teamcode.opmodes.teleop.CombinedTeleOp.Companion.follower
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.FiniteStateMachine
import org.firstinspires.ftc.teamcode.util.FollowPathState
import org.firstinspires.ftc.teamcode.util.WaitState

@Suppress("Unused")
@Autonomous(name = "NEAR 12 Ball", preselectTeleOp = "Combined TeleOp")
class Near12AutonomousSimple: BaseAutonomous() {
    override fun initialize(alliance: Alliance) {
        pathSequence = Near12(alliance)
        paths = pathSequence.buildPaths(follower!!)

        finiteStateMachine = FiniteStateMachine(
            FollowPathState("Score preload", paths[0]),
            getMotifState(),
            startIntake(),
            WaitState(250.0),
            shootState(),
            FollowPathState("Dintake 1", paths[1]),
            FollowPathState("intake 1", paths[2]),
            FollowPathState("dscore1", paths[3]),
            shootState(),
            FollowPathState("Dintake 2", paths[4]),
            FollowPathState("Intake 2", paths[5]),
            FollowPathState("DScore 3", paths[6]),
            shootState(),
            FollowPathState("Dintake 3", paths[7]),
            FollowPathState("Intake 3", paths[8]),
            FollowPathState("DScore 3", paths[9]),
            shootState(),
            FollowPathState("Park", paths[10])
        )
    }
}
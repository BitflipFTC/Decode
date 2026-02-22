package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.auto.paths.GarnetSquadron
import org.firstinspires.ftc.teamcode.opmodes.teleop.CombinedTeleOp
import org.firstinspires.ftc.teamcode.opmodes.teleop.CombinedTeleOp.Companion.follower
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.FiniteStateMachine
import org.firstinspires.ftc.teamcode.util.FollowPathState
import org.firstinspires.ftc.teamcode.util.WaitState

@Suppress("Unused")
@Autonomous(name = "Garnet Squadron Partner", preselectTeleOp = "Combined TeleOp")
class GarnetSquadronAutonomous: BaseAutonomous() {
    override fun initialize(alliance: Alliance) {
        pathSequence = GarnetSquadron(alliance)
        paths = pathSequence.buildPaths(follower!!)
        follower!!.setStartingPose(pathSequence.poses.farStartPose)

        finiteStateMachine = FiniteStateMachine(
            FollowPathState("Score preload", paths[0]),
            startIntake(),
            WaitState(100.0),
            shootState(),
            FollowPathState("D HP & Intake", paths[1]),
            WaitState(150.0),
            FollowPathState("D Score", paths[2]),
            shootState(),
            FollowPathState("dtointake", paths[3]),
            WaitState(1500.0),
            FollowPathState("do the intake", paths[4]),
            WaitState(1000.0),
            FollowPathState("d to shoito", paths[5]),
            shootState(),
            FollowPathState("Park", paths[6])
        )
    }
}
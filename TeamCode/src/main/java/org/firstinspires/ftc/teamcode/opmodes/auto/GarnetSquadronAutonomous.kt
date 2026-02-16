package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.auto.paths.GarnetSquadron
import org.firstinspires.ftc.teamcode.opmodes.teleop.CombinedTeleOp
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.FiniteStateMachine
import org.firstinspires.ftc.teamcode.util.FollowPathState
import org.firstinspires.ftc.teamcode.util.WaitState


@Suppress("Unused")
@Autonomous(name = "Garnet Squadron Partner", preselectTeleOp = "Combined TeleOp")
class GarnetSquadronAutonomous: BaseAutonomous() {
    override fun initialize(alliance: Alliance) {
        pathSequence = GarnetSquadron(alliance)
        paths = pathSequence.buildPaths(CombinedTeleOp.follower!!)

        finiteStateMachine = FiniteStateMachine(
            FollowPathState("Score preload", paths[0]),
            startIntake(),
            getMotifState(),
            WaitState(100.0),
            shootState(),
            FollowPathState("D HP", paths[1]),
            FollowPathState("HP Intake", paths[2]),
            FollowPathState("D Score", paths[3]),
            shootState(),
            FollowPathState("Park", paths[4])
        )
    }
}
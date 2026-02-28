package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.auto.paths.RaptoriaAlliance
import org.firstinspires.ftc.teamcode.opmodes.teleop.CombinedTeleOp.Companion.follower
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.FiniteStateMachine
import org.firstinspires.ftc.teamcode.util.FollowPathState
import org.firstinspires.ftc.teamcode.util.InitializeState
import org.firstinspires.ftc.teamcode.util.WaitState

@Suppress("Unused")
@Autonomous(name = "Garnet Squadron Partner", preselectTeleOp = "Combined TeleOp")
class RaptoriaAllianceAutonomous: BaseAutonomous() {
    override fun initialize(alliance: Alliance) {
        pathSequence = RaptoriaAlliance(alliance)
        paths = pathSequence.buildPaths(follower!!)
        follower!!.setStartingPose(pathSequence.poses.farStartPose)

        finiteStateMachine = FiniteStateMachine(
            FollowPathState("Score preload", paths[0]),
            startIntake(),
            shootState(),

            FollowPathState("D HP", paths[1]),
            FollowPathState("Intake", paths[2]),
            WaitState(150.0),
            FollowPathState("D Score", paths[3]),
            shootState(),

            FollowPathState("dtointake", paths[4]),
            InitializeState("zeep", { !follower!!.isBusy }, { follower!!.turnTo(0.0)}),
            WaitState(500.0),
            FollowPathState("do the intake", paths[5]),
            WaitState(500.0),
            FollowPathState("d to shoito", paths[6]),
            shootState(),

            FollowPathState("dtointake2", paths[7]),
            InitializeState("zeep", { follower!!.isBusy }, { follower!!.turnTo(0.0)}),
            WaitState(500.0),
            FollowPathState("do the intake2", paths[8]),
            WaitState(500.0),
            FollowPathState("d to shoito2", paths[9]),
            shootState(),

            FollowPathState("dtointake3", paths[10]),
            InitializeState("zeep", { follower!!.isBusy }, { follower!!.turnTo(0.0)}),
            WaitState(500.0),
            FollowPathState("do the intake3", paths[11]),
            WaitState(500.0),
            FollowPathState("d to shoito3", paths[12]),
            shootState(),

            FollowPathState("Park", paths[13])
        )
    }
}
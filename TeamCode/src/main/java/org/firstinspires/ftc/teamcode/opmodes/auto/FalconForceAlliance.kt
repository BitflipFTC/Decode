package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.auto.paths.FalconForce
import org.firstinspires.ftc.teamcode.opmodes.teleop.CombinedTeleOp.Companion.follower
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.FiniteStateMachine
import org.firstinspires.ftc.teamcode.util.TimedFollowPathState
import org.firstinspires.ftc.teamcode.util.WaitState

@Suppress("Unused")
@Autonomous(name = "Falcon Force Partner", preselectTeleOp = "Combined TeleOp")
class FalconForceAlliance: BaseAutonomous() {
    override fun initialize(alliance: Alliance) {
        pathSequence = FalconForce(alliance)
        paths = pathSequence.buildPaths(follower!!)
        follower!!.setStartingPose(pathSequence.poses.farStartPose)

        finiteStateMachine = FiniteStateMachine(
            TimedFollowPathState("Score preload", paths[0], 2000.0),
            startIntake(),
            shootState(),

            WaitState(5000.0),

            TimedFollowPathState("D HP", paths[1], 2000.0),
            TimedFollowPathState("Intake", paths[2], 3000.0),
            WaitState(150.0),
            TimedFollowPathState("D Score", paths[3], 2000.0),
            shootState(),

            TimedFollowPathState("dtointake", paths[4], 2000.0),
//            InitializeState("zeep", { !follower!!.isBusy }, { follower!!.turnTo(0.0)}),
            TimedFollowPathState("do the intake", paths[5], 3000.0),
            TimedFollowPathState("d to shoito", paths[6], 2000.0),
            shootState(),

            TimedFollowPathState("dtointake2", paths[7], 2000.0),
//            InitializeState("zeep", { !follower!!.isBusy }, { follower!!.turnTo(0.0)}),
            TimedFollowPathState("do the intake2", paths[8], 3000.0),
            TimedFollowPathState("d to shoito2", paths[9], 2000.0),
            shootState(),

//            TimedFollowPathState("dtointake3", paths[10], 2000.0),
//            InitializeState("zeep", { !follower!!.isBusy }, { follower!!.turnTo(0.0)}),
//            TimedFollowPathState("do the intake3", paths[11], 3000.0),
//            TimedFollowPathState("d to shoito3", paths[12], 2000.0),
//            relocalizeState(),
//            shootState(),

            TimedFollowPathState("Park", paths[13], 2000.0)
        )
    }
}
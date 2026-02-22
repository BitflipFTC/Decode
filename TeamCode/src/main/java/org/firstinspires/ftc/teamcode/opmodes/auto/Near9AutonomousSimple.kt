package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.auto.paths.Near9
import org.firstinspires.ftc.teamcode.opmodes.teleop.CombinedTeleOp.Companion.follower
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.FiniteStateMachine
import org.firstinspires.ftc.teamcode.util.FollowPathState
import org.firstinspires.ftc.teamcode.util.WaitState

@Suppress("Unused")
@Autonomous(name = "NEAR 9 Ball", preselectTeleOp = "Combined TeleOp")
class Near9AutonomousSimple: BaseAutonomous() {
    override fun initialize(alliance: Alliance) {
        //            Path(scorePreload, fullSpeed),
        //            Path(dintake1, fullSpeed),
        //            Path(intake1, intakeSpeed),
        //            Path(score1, fullSpeed),
        //            Path(dintake2, fullSpeed),
        //            Path(intake2, intakeSpeed),
        //            Path(score2, fullSpeed),
        //            Path(park, fullSpeed)
        pathSequence = Near9(alliance)
        paths = pathSequence.buildPaths(follower!!)
        follower!!.setStartingPose(pathSequence.poses.nearStartPose)

        finiteStateMachine = FiniteStateMachine(
            FollowPathState("D Score Preload", paths[0]),
            startIntake(),
            shootState(),

            FollowPathState("Dintake 1", paths[1]),
            FollowPathState("Intake 1", paths[2]),
            WaitState(150.0),
            FollowPathState("D Score 1", paths[3]),
            shootState(),

            FollowPathState("Dintake 2", paths[4]),
            FollowPathState("Intake 2", paths[5]),
            WaitState(150.0),
            FollowPathState("D Score 2", paths[6]),
            shootState(),

            FollowPathState("Park", paths[7])
        )
    }
}
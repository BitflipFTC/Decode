package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.auto.paths.Near9
import org.firstinspires.ftc.teamcode.opmodes.teleop.CombinedTeleOp.Companion.follower
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.FiniteStateMachine
import org.firstinspires.ftc.teamcode.util.FollowPathState
import org.firstinspires.ftc.teamcode.util.InstantState
import org.firstinspires.ftc.teamcode.util.TimedFollowPathState
import org.firstinspires.ftc.teamcode.util.WaitState

@Suppress("Unused")
@Autonomous(name = "NEAR 9 Ball", preselectTeleOp = "Combined TeleOp")
class Near9AutonomousSimple: BaseAutonomous() {
    override fun initialize(alliance: Alliance) {
        pathSequence = Near9(alliance)
        paths = pathSequence.buildPaths(follower!!)
        follower!!.setStartingPose(pathSequence.poses.nearStartPose)

        finiteStateMachine = FiniteStateMachine(
            InstantState("zeep", { turret.automatic = false
                if (alliance== Alliance.RED) turret.angle = 120.0 else turret.angle = -120.0} ),
            startIntake(),
            FollowPathState("D Score Preload", paths[0]),
            InstantState("aaaa",{turret.automatic = true}),
            WaitState(759.0),
            shootState(),

            TimedFollowPathState("Dintake 1", paths[1], 5000.0),
            TimedFollowPathState("Intake 1", paths[2], 2000.0),
            WaitState(150.0),
            TimedFollowPathState("D Score 1", paths[3], 5000.0),
            shootState(),

            TimedFollowPathState("uhhhhh", paths[4], 10000.0),

            FollowPathState("Park", paths[5])
        )
    }
}
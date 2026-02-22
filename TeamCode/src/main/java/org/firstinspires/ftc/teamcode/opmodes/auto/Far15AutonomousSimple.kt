package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.auto.paths.Far15
import org.firstinspires.ftc.teamcode.opmodes.teleop.CombinedTeleOp
import org.firstinspires.ftc.teamcode.opmodes.teleop.CombinedTeleOp.Companion.follower
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.FiniteStateMachine
import org.firstinspires.ftc.teamcode.util.FollowPathState
import org.firstinspires.ftc.teamcode.util.WaitState

@Suppress("Unused")
@Autonomous(name = "FAR 15 Ball", preselectTeleOp = "Combined TeleOp")
class Far15AutonomousSimple: BaseAutonomous() {
    override fun initialize(alliance: Alliance) {
        pathSequence = Far15(alliance)
        paths = pathSequence.buildPaths(CombinedTeleOp.follower!!)
        follower!!.setStartingPose(pathSequence.poses.farStartPose)

        finiteStateMachine = FiniteStateMachine(
            FollowPathState("D Score Preload", paths[0]),
            startIntake(),
            shootState(),

            FollowPathState("D Intake 3", paths[1]),
            FollowPathState("intake 3", paths[2]),
            WaitState(150.0),
            FollowPathState("D Score 3", paths[3]),
            shootState(),

            FollowPathState("D Intake 2", paths[4]),
            FollowPathState("Intake 2", paths[5]),
            WaitState(150.0),
            FollowPathState("d empty ramp", paths[6]),
            FollowPathState("Empty Ramp", paths[7]),
            WaitState(250.0),
            FollowPathState("D Score 2", paths[8]),
            shootState(),

            FollowPathState("D Intake 1", paths[9]),
            FollowPathState("intake 1", paths[10]),
            WaitState(150.0),
            FollowPathState("D Score 1", paths[11]),
            shootState(),

            FollowPathState("D hp intake", paths[12]),
            FollowPathState("HP INTAKE", paths[13]),
            WaitState(250.0),
            FollowPathState("D Score HP", paths[14]),
            shootState(),

            FollowPathState("Park", paths[15])
        )
    }
}
package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.auto.paths.Far12
import org.firstinspires.ftc.teamcode.opmodes.teleop.CombinedTeleOp
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.FiniteStateMachine
import org.firstinspires.ftc.teamcode.util.FollowPathState
import org.firstinspires.ftc.teamcode.util.WaitState

@Suppress("Unused")
@Autonomous(name = "FAR 12 Ball", preselectTeleOp = "Combined TeleOp")
class Far12AutonomousSimple: BaseAutonomous() {
    override fun initialize(alliance: Alliance) {
        pathSequence = Far12(alliance)
        paths = pathSequence.buildPaths(CombinedTeleOp.follower!!)

        finiteStateMachine = FiniteStateMachine(
            FollowPathState("D Score Preload", paths[0]),
            startIntake(),
            WaitState(250.0),
            shootState(),
            FollowPathState("D Intake 2", paths[1]),
            WaitState(150.0),
            FollowPathState("Empty Ramp", paths[2]),
            WaitState(250.0),
            FollowPathState("D Score 2", paths[3]),
            shootState(),
            FollowPathState("D Intake 3", paths[4]),
            WaitState(150.0),
            FollowPathState("D Score 3", paths[5]),
            shootState(),
            FollowPathState("HP INTAKE", paths[6]),
            WaitState(150.0),
            FollowPathState("D Score HP", paths[7]),
            shootState(),
            FollowPathState("Park", paths[8])
        )
    }
}
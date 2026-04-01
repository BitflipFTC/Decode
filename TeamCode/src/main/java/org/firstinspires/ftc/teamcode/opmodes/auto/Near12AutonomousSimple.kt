package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.auto.paths.Near12
import org.firstinspires.ftc.teamcode.opmodes.teleop.CombinedTeleOp.Companion.follower
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.FiniteStateMachine
import org.firstinspires.ftc.teamcode.util.FollowPathState
import org.firstinspires.ftc.teamcode.util.InitializeState
import org.firstinspires.ftc.teamcode.util.InstantState
import org.firstinspires.ftc.teamcode.util.WaitState
import org.firstinspires.ftc.teamcode.util.WaitUntilState

@Suppress("Unused")
@Autonomous(name = "NEAR 12 Ball", preselectTeleOp = "Combined TeleOp")
class Near12AutonomousSimple: BaseAutonomous() {
    override fun initialize(alliance: Alliance) {
        pathSequence = Near12(alliance)
        paths = pathSequence.buildPaths(follower!!)

        follower!!.setStartingPose(pathSequence.poses.nearStartPose)

        finiteStateMachine = FiniteStateMachine(
            InstantState("zeep", { turret.automatic = false
            turret.angle = 90.0} ),
            FollowPathState("Score preload", paths[0]),
            startIntake(),
//            WaitUntilState( { spindexer.motifPattern != null }),
            InstantState("aaaa",{turret.automatic = true}),
            WaitState(400.0),
            WaitUntilState(shooter::atSetPoint),
            shootState(),

            FollowPathState("dintake 1", paths[1]),
            WaitState(150.0),
            FollowPathState("dramping it", paths[2]),
            FollowPathState("ramping ", paths[3], 750.0),
            FollowPathState("dscore1", paths[4]),
            shootState(),

            FollowPathState("dintake 2", paths[5]),
            WaitState(150.0),

            FollowPathState("DScore 3", paths[6]),
            shootState(),

            FollowPathState("dIntake 3", paths[7]),
            WaitState(150.0),
            FollowPathState("DScore 3", paths[8]),
            shootState(),
        )
    }


}
package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.utility.InstantCommand
import org.firstinspires.ftc.teamcode.opmodes.auto.paths.Far12
import org.firstinspires.ftc.teamcode.opmodes.teleop.CombinedTeleOp
import org.firstinspires.ftc.teamcode.opmodes.teleop.CombinedTeleOp.Companion.follower
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.FiniteStateMachine
import org.firstinspires.ftc.teamcode.util.FollowPathState
import org.firstinspires.ftc.teamcode.util.InstantState
import org.firstinspires.ftc.teamcode.util.TimedFollowPathState
import org.firstinspires.ftc.teamcode.util.WaitState
import kotlin.time.TimedValue

@Suppress("Unused")
@Autonomous(name = "FAR 12 Ball", preselectTeleOp = "Combined TeleOp")
class Far12AutonomousSimple: BaseAutonomous() {
    override fun initialize(alliance: Alliance) {
        //             Path(scorePreload, fullSpeed),
        //            Path(dintake2, fullSpeed),
        //            Path(intake2, intakeSpeed),
        //            Path(demptyRamp, fullSpeed),
        //            Path(emptyRamp, intakeSpeed),
        //            Path(score2, fullSpeed),
        //            Path(dintake3, fullSpeed),
        //            Path(intake3, intakeSpeed),
        //            Path(score3, fullSpeed),
        //            Path(dHP, fullSpeed),
        //            Path(HP, 0.25),
        //            Path(dScoreHP, fullSpeed),
        //            Path(park, fullSpeed)

        pathSequence = Far12(alliance)
        paths = pathSequence.buildPaths(CombinedTeleOp.follower!!)
        follower!!.setStartingPose(pathSequence.poses.farStartPose)

        finiteStateMachine = FiniteStateMachine(
            FollowPathState("D Score Preload", paths[0]),
            startIntake(),
            shootState(),

            FollowPathState("D Intake 2", paths[1]),
            FollowPathState("Intake 2", paths[2]),
            WaitState(150.0),
            FollowPathState("d empty ramp", paths[3]),
            TimedFollowPathState("Empty Ramp", paths[4], 2000.0),
            WaitState(250.0),
            FollowPathState("D Score 2", paths[5]),
            shootState(),

            FollowPathState("D Intake 3", paths[6]),
            FollowPathState("intake 3", paths[7]),
            WaitState(150.0),
            FollowPathState("D Score 3", paths[8]),
            shootState(),

            FollowPathState("D hp intake", paths[9]),
            FollowPathState("HP INTAKE", paths[10]),
            WaitState(150.0),
            FollowPathState("D Score HP", paths[11]),
            shootState(),

            FollowPathState("Park", paths[12])
        )
    }
}
package org.firstinspires.ftc.teamcode.opmodes.auto.paths

import com.pedropathing.follower.Follower
import com.pedropathing.paths.PathChain
import com.pedropathing.paths.callbacks.ParametricCallback
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAutonomous.Companion.PARAMETRIC_END
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.auto.BaseAutoPath
import org.firstinspires.ftc.teamcode.util.auto.Path
import org.firstinspires.ftc.teamcode.util.buildBasicLine
import org.firstinspires.ftc.teamcode.util.buildCurvedLine
import org.firstinspires.ftc.teamcode.util.buildCurvedTangentLine
import org.firstinspires.ftc.teamcode.util.buildIntakeLine
import org.firstinspires.ftc.teamcode.util.buildTangentLine
import org.firstinspires.ftc.teamcode.util.doIntakeSpeed

class Near12(alliance: Alliance) : BaseAutoPath(alliance) {
    val fullSpeed = 1.0
    val intakeSpeed = 0.85

    lateinit var scorePreload: PathChain
    lateinit var dintake1: PathChain
    lateinit var demptyRamp: PathChain
    lateinit var emptyRamp: PathChain
    lateinit var score1: PathChain
    lateinit var dintake2: PathChain
    lateinit var score2: PathChain
    lateinit var dintake3: PathChain
    lateinit var score3: PathChain

    override fun buildPaths(follower: Follower): List<Path> {
        scorePreload = follower.pathBuilder()
            .buildBasicLine(poses.nearStartPose, poses.nearShootPose).build()

        dintake1 = follower.pathBuilder()
            .buildCurvedTangentLine(poses.nearShootPose, poses.nearIntake2Control, poses.endIntake2)
//            .buildTangentLine(poses.startIntake2, poses.endIntake2)
            .build()

        demptyRamp = follower.pathBuilder()
            .buildCurvedLine(poses.endIntake2, poses.endIntake2Move, poses.emptyRampStart).build()

        emptyRamp = follower.pathBuilder()
            .buildBasicLine(poses.emptyRampStart, poses.emptyRamp)
            .setTValueConstraint(0.6)
            .build()

        score1 = follower.pathBuilder()
            .buildCurvedLine(poses.endIntake2, poses.endIntake2Move, poses.nearShootPose).build()

        dintake2 = follower.pathBuilder()
            .buildBasicLine(poses.nearShootPose,poses.startIntake1)
            .buildTangentLine(poses.startIntake1, poses.endIntake1)
            .build()

        score2 = follower.pathBuilder()
            .buildBasicLine(poses.endIntake1, poses.nearShootPose).build()

        dintake3 = follower.pathBuilder()
            .buildCurvedTangentLine(poses.nearShootPose,  poses.nearIntake3Control, poses.endIntake3)
//            .buildTangentLine(poses.startIntake3, poses.endIntake3)
            .build()

        score3 = follower.pathBuilder()
            .buildCurvedTangentLine(poses.endIntake3, poses.endIntake3Move, poses.nearShootPoseInZone).setReversed().build()

        return listOf(
            Path(scorePreload, fullSpeed),
            Path(dintake1, fullSpeed),
            Path(demptyRamp, fullSpeed),
            Path(emptyRamp, intakeSpeed),
            Path(score1, fullSpeed),
            Path(dintake2, fullSpeed),
            Path(score2, fullSpeed),
            Path(dintake3, fullSpeed),
            Path(score3, fullSpeed),
            )
    }
}
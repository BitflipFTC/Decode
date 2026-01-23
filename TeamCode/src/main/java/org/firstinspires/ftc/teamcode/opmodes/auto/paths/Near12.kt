package org.firstinspires.ftc.teamcode.opmodes.auto.paths

import com.pedropathing.follower.Follower
import com.pedropathing.paths.PathChain
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.auto.BaseAutoPath
import org.firstinspires.ftc.teamcode.util.auto.Path
import org.firstinspires.ftc.teamcode.util.buildBasicLine
import org.firstinspires.ftc.teamcode.util.buildCurvedLine
import org.firstinspires.ftc.teamcode.util.buildTangentLine

class Near12 (
    alliance: Alliance
) : BaseAutoPath(
    alliance
) {
    val fullSpeed = 1.0
    val intakeSpeed = 0.5

    lateinit var scorePreload: PathChain
    lateinit var dIntake1: PathChain
    lateinit var intake1: PathChain
    lateinit var score1: PathChain
    lateinit var dIntake2: PathChain
    lateinit var intake2: PathChain
    lateinit var score2: PathChain
    lateinit var dIntake3: PathChain
    lateinit var intake3: PathChain
    lateinit var score3: PathChain
    lateinit var park: PathChain

    override fun buildPaths(follower: Follower): List<Path> {
        scorePreload = follower.pathBuilder()
            .buildBasicLine(poses.nearStartPose, poses.nearShootPoseFacingObelisk).build()

        dIntake1 = follower.pathBuilder()
            .buildCurvedLine(poses.nearShootPoseFacingObelisk, poses.nearIntake1Control, poses.startIntake1).build()

        intake1 = follower.pathBuilder()
            .buildTangentLine(poses.startIntake1, poses.endIntake1).build()

        score1 = follower.pathBuilder()
            .buildBasicLine(poses.endIntake1, poses.nearShootPose).build()

        dIntake2 = follower.pathBuilder()
            .buildCurvedLine(poses.nearShootPose, poses.nearIntake2Control, poses.startIntake2).build()

        intake2 = follower.pathBuilder()
            .buildTangentLine(poses.startIntake2, poses.endIntake2).build()

        score2 = follower.pathBuilder()
            .buildTangentLine(poses.endIntake2, poses.endIntake2Move).setReversed()
            .buildBasicLine(poses.endIntake2Move, poses.nearShootPose).build()

        dIntake3 = follower.pathBuilder()
            .buildCurvedLine(poses.nearShootPose, poses.nearIntake3Control, poses.startIntake3).build()

        intake3 = follower.pathBuilder()
            .buildTangentLine(poses.startIntake3, poses.endIntake3).build()

        score3 = follower.pathBuilder()
            .buildTangentLine(poses.endIntake3, poses.endIntake3Move).setReversed()
            .buildBasicLine(poses.endIntake3, poses.nearShootPose).build()

        park = follower.pathBuilder()
            .buildBasicLine(poses.nearShootPose, poses.nearParkPose).build()

        return listOf(
            Path(scorePreload, fullSpeed),
            Path(dIntake1, fullSpeed),
            Path(intake1, intakeSpeed),
            Path(score1, fullSpeed),
            Path(dIntake2, fullSpeed),
            Path(intake2, intakeSpeed),
            Path(score2, fullSpeed),
            Path(dIntake3, fullSpeed),
            Path(intake3, intakeSpeed),
            Path(score3, fullSpeed),
            Path(park, fullSpeed)
            )
    }
}
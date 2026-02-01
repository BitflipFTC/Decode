package org.firstinspires.ftc.teamcode.opmodes.auto.paths

import com.pedropathing.follower.Follower
import com.pedropathing.paths.PathChain
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.auto.BaseAutoPath
import org.firstinspires.ftc.teamcode.util.auto.Path
import org.firstinspires.ftc.teamcode.util.buildBasicLine
import org.firstinspires.ftc.teamcode.util.buildCurvedLine
import org.firstinspires.ftc.teamcode.util.buildTangentLine

class Near9 (
    alliance: Alliance
) : BaseAutoPath(
    alliance
) {
    val fullSpeed = 1.0
    val intakeSpeed = 0.3

    lateinit var scorePreload: PathChain
    lateinit var dIntake1: PathChain
    lateinit var intake1: PathChain
    lateinit var score1: PathChain
    lateinit var dIntake2: PathChain
    lateinit var intake2: PathChain
    lateinit var score2: PathChain
    lateinit var park: PathChain

    override fun buildPaths(follower: Follower): List<Path> {
        scorePreload = follower.pathBuilder()
            .buildBasicLine(poses.nearStartPose, poses.nearShootPoseFacingObelisk).build()

        dIntake1 = follower.pathBuilder()
            .buildCurvedLine(poses.nearShootPoseFacingObelisk, poses.nearIntake1Control, poses.startIntake1).build()

        intake1 = follower.pathBuilder()
            .buildTangentLine(poses.startIntake1, poses.endIntake1).build()

        score1 = follower.pathBuilder()
            .buildTangentLine(poses.endIntake1, poses.nearShootPose).setReversed().build()

        dIntake2 = follower.pathBuilder()
            .buildCurvedLine(poses.nearShootPose, poses.nearIntake2Control, poses.startIntake2).build()

        intake2 = follower.pathBuilder()
            .buildTangentLine(poses.startIntake2, poses.endIntake2).build()

        score2 = follower.pathBuilder()
            .buildCurvedLine(poses.endIntake2, poses.endIntake2Move, poses.nearShootPose).build()

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
            Path(park, fullSpeed)
        )
    }
}
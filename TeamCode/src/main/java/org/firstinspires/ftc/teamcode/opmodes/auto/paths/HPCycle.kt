package org.firstinspires.ftc.teamcode.opmodes.auto.paths

import com.pedropathing.follower.Follower
import com.pedropathing.paths.PathChain
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.auto.BaseAutoPath
import org.firstinspires.ftc.teamcode.util.auto.Path
import org.firstinspires.ftc.teamcode.util.buildBasicLine
import org.firstinspires.ftc.teamcode.util.buildCurvedLine
import org.firstinspires.ftc.teamcode.util.buildTangentLine

class HPCycle(alliance: Alliance): BaseAutoPath(alliance) {

    val fullSpeed = 0.9
    val intakeSpeed = 0.7

    lateinit var scorePreload: PathChain
    lateinit var dHP: PathChain
    lateinit var dScore: PathChain
    lateinit var dhp1: PathChain
    lateinit var dScoreGate: PathChain
    lateinit var dhp2: PathChain
    lateinit var dscore2: PathChain
    lateinit var park: PathChain

    override fun buildPaths(follower: Follower): List<Path> {
        scorePreload = follower.pathBuilder()
                .buildBasicLine(poses.farStartPose, poses.farShootPose).build()

        dHP = follower.pathBuilder()
            .buildTangentLine(poses.farShootPose, poses.HPIntakeStart)
            .buildBasicLine(poses.HPIntakeStart, poses.HPIntakeEnd).build()

        dScore = follower.pathBuilder()
            .buildBasicLine(poses.HPIntakeEnd, poses.farShootPose).build()

        dhp1 = follower.pathBuilder()
            .buildTangentLine(poses.farShootPose, poses.HPPark)
            .buildTangentLine(poses.HPIntakeStart, poses.HPIntakeEnd).build()

        dScoreGate = follower.pathBuilder()
            .buildBasicLine(poses.HPIntakeEnd, poses.farShootPose).build()

        dhp2 = follower.pathBuilder()
            .buildBasicLine(poses.farShootPose, poses.AboveHPIntakeStart)
            .buildBasicLine(poses.AboveHPIntakeStart, poses.AboveHPIntakeEnd).build()

        dscore2 = follower.pathBuilder()
            .buildBasicLine(poses.AboveHPIntakeEnd, poses.farShootPose).build()

        park = follower.pathBuilder()
            .buildBasicLine(poses.farShootPose, poses.HPPark).build()

        return listOf(
            Path(scorePreload, fullSpeed),
            Path(dHP, intakeSpeed),
            Path(dScore, fullSpeed),
            Path(dhp1, intakeSpeed),
            Path(dScoreGate, fullSpeed),
            Path(dhp2, intakeSpeed),
            Path(dscore2, fullSpeed),
            Path(park, fullSpeed)
        )
    }
}
package org.firstinspires.ftc.teamcode.opmodes.auto.paths

import com.pedropathing.follower.Follower
import com.pedropathing.paths.PathChain
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.auto.BaseAutoPath
import org.firstinspires.ftc.teamcode.util.auto.Path
import org.firstinspires.ftc.teamcode.util.buildBasicLine
import org.firstinspires.ftc.teamcode.util.buildTangentLine

class GarnetSquadron(alliance: Alliance): BaseAutoPath(alliance) {

    val fullSpeed = 1.0
    val intakeSpeed = 0.67

    lateinit var scorePreload: PathChain
    lateinit var dHP: PathChain
    lateinit var HPintake: PathChain
    lateinit var dScore: PathChain
    lateinit var park: PathChain

    override fun buildPaths(follower: Follower): List<Path> {
        scorePreload = follower.pathBuilder()
            .buildBasicLine(poses.farStartPose, poses.farShootPoseFacingObelisk).build()

        dHP = follower.pathBuilder()
            .buildTangentLine(poses.farShootPoseFacingObelisk, poses.HPIntakeStart).build()

        HPintake = follower.pathBuilder()
            .buildTangentLine(poses.HPIntakeStart, poses.HPIntakeEnd).build()

        dScore = follower.pathBuilder()
            .buildTangentLine(poses.HPIntakeEnd, poses.farShootPose).build()

        park = follower.pathBuilder()
            .buildBasicLine(poses.farShootPose, poses.farParkPose).build()

        return listOf(
            Path(scorePreload, fullSpeed),
            Path(dHP, fullSpeed),
            Path(HPintake, intakeSpeed),
            Path(dScore, fullSpeed),
            Path(park, fullSpeed)
        )
    }
}
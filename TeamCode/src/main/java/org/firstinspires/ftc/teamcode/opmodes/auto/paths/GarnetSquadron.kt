package org.firstinspires.ftc.teamcode.opmodes.auto.paths

import com.pedropathing.follower.Follower
import com.pedropathing.paths.PathChain
import com.pedropathing.paths.callbacks.ParametricCallback
import com.pedropathing.paths.callbacks.PoseCallback
import com.pedropathing.paths.callbacks.TemporalCallback
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
    lateinit var dScore: PathChain
    lateinit var park: PathChain

    override fun buildPaths(follower: Follower): List<Path> {
        scorePreload = follower.pathBuilder()
            .buildBasicLine(poses.farStartPose, poses.farShootPoseFacingObelisk).build()

        dHP = follower.pathBuilder()
            .buildTangentLine(poses.farShootPoseFacingObelisk, poses.HPIntakeStart)
            .buildTangentLine(poses.HPIntakeStart, poses.HPIntakeEnd).build()

        dHP.setCallbacks(
            ParametricCallback(0, 0.98, follower, {follower.setMaxPower(intakeSpeed)}),
            ParametricCallback(1, 0.98, follower,{follower.setMaxPower(fullSpeed)})
        )

        dScore = follower.pathBuilder()
            .buildTangentLine(poses.HPIntakeEnd, poses.farShootPose).build()

        park = follower.pathBuilder()
            .buildBasicLine(poses.farShootPose, poses.farParkPose).build()

        return listOf(
            Path(scorePreload, fullSpeed),
            Path(dHP, fullSpeed),
            Path(dScore, fullSpeed),
            Path(park, fullSpeed)
        )
    }
}
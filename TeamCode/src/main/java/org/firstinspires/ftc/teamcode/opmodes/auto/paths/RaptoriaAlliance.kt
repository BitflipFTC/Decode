package org.firstinspires.ftc.teamcode.opmodes.auto.paths

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.paths.PathChain
import com.pedropathing.paths.callbacks.ParametricCallback
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.auto.BaseAutoPath
import org.firstinspires.ftc.teamcode.util.auto.Path
import org.firstinspires.ftc.teamcode.util.buildBasicLine
import org.firstinspires.ftc.teamcode.util.buildTangentLine

class RaptoriaAlliance(alliance: Alliance): BaseAutoPath(alliance) {

    val fullSpeed = 0.8
    val intakeSpeed = 0.5

    lateinit var scorePreload: PathChain
    lateinit var dHP: PathChain
    lateinit var HP: PathChain
    lateinit var dScore: PathChain
    lateinit var dIntakeGate: PathChain
    lateinit var intakeGate: PathChain
    lateinit var dScoreGate: PathChain
    lateinit var dIntakeGate2: PathChain
    lateinit var intakeGate2: PathChain
    lateinit var dScoreGate2: PathChain
    lateinit var park: PathChain

    override fun buildPaths(follower: Follower): List<Path> {
        scorePreload = follower.pathBuilder()
                .buildBasicLine(poses.farStartPose, poses.farShootPoseFacingObelisk).setTValueConstraint(0.1).build()
        scorePreload.setCallbacks(ParametricCallback(
            0, 0.1, follower, follower::breakFollowing
        ))

        dHP = follower.pathBuilder()
            .buildTangentLine(poses.farShootPoseFacingObelisk, poses.HPIntakeStart).build()

        HP = follower.pathBuilder()
            .buildTangentLine(poses.HPIntakeStart, poses.HPIntakeEnd).build()

        dScore = follower.pathBuilder()
            .buildTangentLine(poses.HPIntakeEnd, poses.farShootPose).setReversed().build()

        dIntakeGate = follower.pathBuilder()
            .buildTangentLine(poses.farShootPose, poses.HPPark).build()

        intakeGate = follower.pathBuilder()
            .addPath(BezierLine(poses.HPPark, poses.HPIntakeEnd))
            .setLinearHeadingInterpolation(poses.HPPark.heading, Math.toRadians(330.0), 1.0, 0.0)
            .build()

        dScoreGate = follower.pathBuilder()
            .buildTangentLine(poses.HPIntakeEnd, poses.farShootPose).setReversed().build()

        dIntakeGate2 = follower.pathBuilder()
            .buildTangentLine(poses.farShootPose, poses.HPPark).build()

        intakeGate2 = follower.pathBuilder()
            .addPath(BezierLine(poses.HPPark, poses.HPIntakeEnd))
            .setLinearHeadingInterpolation(poses.HPPark.heading, Math.toRadians(330.0), 1.0, 0.0)
            .build()

        dScoreGate2 = follower.pathBuilder()
            .buildTangentLine(poses.HPIntakeEnd, poses.farShootPose).setReversed().build()

        park = follower.pathBuilder()
            .buildTangentLine(poses.farShootPose, poses.HPPark).setReversed().build()

        return listOf(
            Path(scorePreload, fullSpeed),
            Path(dHP, intakeSpeed),
            Path(HP, intakeSpeed),
            Path(dScore, fullSpeed),
            Path(dIntakeGate, fullSpeed),
            Path(intakeGate, intakeSpeed),
            Path(dScoreGate, fullSpeed),
            Path(dIntakeGate2, fullSpeed),
            Path(intakeGate2, intakeSpeed),
            Path(dScoreGate2, fullSpeed),
            Path(park, fullSpeed)
        )
    }
}
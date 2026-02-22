package org.firstinspires.ftc.teamcode.opmodes.auto.paths

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.paths.PathChain
import com.pedropathing.paths.callbacks.ParametricCallback
import com.pedropathing.paths.callbacks.PoseCallback
import com.pedropathing.paths.callbacks.TemporalCallback
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAutonomous.Companion.PARAMETRIC_END
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.auto.BaseAutoPath
import org.firstinspires.ftc.teamcode.util.auto.Path
import org.firstinspires.ftc.teamcode.util.buildBasicLine
import org.firstinspires.ftc.teamcode.util.buildCurvedLine
import org.firstinspires.ftc.teamcode.util.buildTangentLine
import org.firstinspires.ftc.teamcode.util.doIntakeSpeed

class GarnetSquadron(alliance: Alliance): BaseAutoPath(alliance) {

    val fullSpeed = 0.8
    val intakeSpeed = 0.5

    lateinit var scorePreload: PathChain
    lateinit var dHP: PathChain
    lateinit var dScore: PathChain
    lateinit var dIntakeGate: PathChain
    lateinit var intakeGate: PathChain
    lateinit var dScore2: PathChain
    lateinit var park: PathChain

    override fun buildPaths(follower: Follower): List<Path> {
        scorePreload = follower.pathBuilder()
                .buildBasicLine(poses.farStartPose, poses.farShootPoseFacingObelisk).setTValueConstraint(0.1).build()
        scorePreload.setCallbacks(ParametricCallback(
            0, 0.1, follower, follower::breakFollowing
        ))

        dHP = follower.pathBuilder()
            .buildTangentLine(poses.farShootPoseFacingObelisk, poses.HPIntakeStart)
            .buildTangentLine(poses.HPIntakeStart, poses.HPIntakeEnd).build()

        dHP.doIntakeSpeed(follower, fullSpeed, intakeSpeed)
        dScore = follower.pathBuilder()
            .buildTangentLine(poses.HPIntakeEnd, poses.farShootPose).setReversed().build()

        dIntakeGate = follower.pathBuilder()
            .buildTangentLine(poses.farShootPose, poses.HPPark).build()

        intakeGate = follower.pathBuilder()
            .addPath(BezierLine(poses.HPPark, poses.HPIntakeEnd))
            .setLinearHeadingInterpolation(poses.HPPark.heading, Math.toRadians(330.0), 1.0, 0.0)
            .build()

        dScore2 = follower.pathBuilder()
            .buildTangentLine(poses.HPIntakeEnd, poses.farShootPose).setReversed().build()

        park = follower.pathBuilder()
            .buildTangentLine(poses.farShootPose, poses.HPPark).setReversed().build()

        return listOf(
            Path(scorePreload, fullSpeed),
            Path(dHP, intakeSpeed),
            Path(dScore, fullSpeed),
            Path(dIntakeGate, fullSpeed),
            Path(intakeGate, intakeSpeed),
            Path(dScore2, fullSpeed),
            Path(park, fullSpeed)
        )
    }
}
package org.firstinspires.ftc.teamcode.opmodes.auto.paths

import com.pedropathing.follower.Follower
import com.pedropathing.paths.PathChain
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.auto.BaseAutoPath
import org.firstinspires.ftc.teamcode.util.auto.Path
import org.firstinspires.ftc.teamcode.util.buildBasicLine
import org.firstinspires.ftc.teamcode.util.buildCurvedLine
import org.firstinspires.ftc.teamcode.util.buildTangentLine

class Far12 (alliance: Alliance) : BaseAutoPath(alliance) {
    val fullSpeed = 1.0
    val intakeSpeed = 0.6

    lateinit var scorePreload: PathChain
    lateinit var intake3: PathChain
    lateinit var score3: PathChain
    lateinit var dHP: PathChain
    lateinit var dScore: PathChain
    lateinit var dhp1: PathChain
    lateinit var dScoreGate: PathChain
    lateinit var park: PathChain

    override fun buildPaths(follower: Follower): List<Path> {
        scorePreload = follower.pathBuilder()
            .buildBasicLine(poses.farStartPose, poses.farShootPose).build()

        intake3 = follower.pathBuilder()
            .buildCurvedLine(poses.farShootPose,poses.farIntake3Control, poses.startIntake3)
            .buildTangentLine(poses.startIntake3,poses.endIntake3).build()

        score3 = follower.pathBuilder()
            .buildCurvedLine(poses.endIntake3, poses.farIntake3Control, poses.farShootPose).build()

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

        park = follower.pathBuilder()
            .buildBasicLine(poses.farShootPose, poses.HPPark).build()

        return listOf(
            Path(scorePreload, fullSpeed),
            Path(intake3,intakeSpeed),
            Path(score3, fullSpeed),
            Path(dHP, intakeSpeed),
            Path(dScore, fullSpeed),
            Path(dhp1, intakeSpeed),
            Path(dScoreGate, fullSpeed),
            Path(park, fullSpeed)
        )
    }
}
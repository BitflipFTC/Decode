package org.firstinspires.ftc.teamcode.opmodes.auto.paths

import com.pedropathing.follower.Follower
import com.pedropathing.paths.PathChain
import com.pedropathing.paths.callbacks.ParametricCallback
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.auto.BaseAutoPath
import org.firstinspires.ftc.teamcode.util.auto.Path
import org.firstinspires.ftc.teamcode.util.buildBasicLine
import org.firstinspires.ftc.teamcode.util.buildCurvedLine
import org.firstinspires.ftc.teamcode.util.buildCurvedTangentLine
import org.firstinspires.ftc.teamcode.util.buildTangentLine

class Near9 (
    alliance: Alliance
) : BaseAutoPath(
    alliance
) {
    val fullSpeed = 1.0
    val intakeSpeed = 0.67

    lateinit var scorePreload: PathChain
    lateinit var intake1: PathChain
    lateinit var score1: PathChain
    lateinit var intake2: PathChain
    lateinit var score2: PathChain
    lateinit var park: PathChain

    override fun buildPaths(follower: Follower): List<Path> {
        scorePreload = follower.pathBuilder()
            .buildBasicLine(poses.nearStartPose, poses.nearShootPoseFacingObelisk).build()

        intake1 = follower.pathBuilder()
            .buildCurvedTangentLine(poses.nearShootPoseFacingObelisk, poses.nearIntake1Control, poses.startIntake1)
            .buildTangentLine(poses.startIntake1, poses.endIntake1).build()

        intake1.setCallbacks(
            ParametricCallback(0, 0.98, follower, {follower.setMaxPower(intakeSpeed)}),
            ParametricCallback(1, 0.98, follower,{follower.setMaxPower(fullSpeed)})
        )

        score1 = follower.pathBuilder()
            .buildTangentLine(poses.endIntake1, poses.nearShootPose).build()

        intake2 = follower.pathBuilder()
            .buildCurvedTangentLine(poses.nearShootPoseFacingObelisk, poses.nearIntake2Control, poses.startIntake2)
            .buildTangentLine(poses.startIntake2, poses.endIntake2).build()

        intake2.setCallbacks(
            ParametricCallback(0, 0.98, follower, {follower.setMaxPower(intakeSpeed)}),
            ParametricCallback(1, 0.98, follower,{follower.setMaxPower(fullSpeed)})
        )

        score2 = follower.pathBuilder()
            .buildCurvedTangentLine(poses.endIntake2, poses.endIntake2Move, poses.nearShootPose).build()

        park = follower.pathBuilder()
            .buildTangentLine(poses.nearShootPose, poses.nearParkPose).build()

        return listOf(
            Path(scorePreload, fullSpeed),
            Path(intake1, fullSpeed),
            Path(score1, fullSpeed),
            Path(intake2, fullSpeed),
            Path(score2, fullSpeed),
            Path(park, fullSpeed)
        )
    }
}
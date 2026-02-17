package org.firstinspires.ftc.teamcode.opmodes.auto.paths

import com.pedropathing.follower.Follower
import com.pedropathing.paths.PathChain
import com.pedropathing.paths.callbacks.ParametricCallback
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.auto.BaseAutoPath
import org.firstinspires.ftc.teamcode.util.auto.Path
import org.firstinspires.ftc.teamcode.util.buildBasicLine
import org.firstinspires.ftc.teamcode.util.buildCurvedTangentLine
import org.firstinspires.ftc.teamcode.util.buildTangentLine

class Far9 (
    alliance: Alliance
) : BaseAutoPath(
    alliance
) {
    val fullSpeed = 1.0
    val intakeSpeed = 0.67

    lateinit var scorePreload: PathChain
    lateinit var intake3: PathChain
    lateinit var score3: PathChain
    lateinit var intake2: PathChain
    lateinit var score2: PathChain
    lateinit var park: PathChain

    override fun buildPaths(follower: Follower): List<Path> {
        scorePreload = follower.pathBuilder()
            .buildBasicLine(poses.farStartPose, poses.farShootPoseFacingObelisk).build()

        intake3 = follower.pathBuilder()
            .buildCurvedTangentLine(poses.farShootPoseFacingObelisk, poses.farIntake3Control, poses.startIntake3)
            .buildTangentLine(poses.startIntake3, poses.endIntake3).build()

        intake3.setCallbacks(
            ParametricCallback(0, 0.98, follower, {follower.setMaxPower(intakeSpeed)}),
            ParametricCallback(1, 0.98, follower,{follower.setMaxPower(fullSpeed)})
        )

        score3 = follower.pathBuilder()
            .buildTangentLine(poses.endIntake3, poses.farShootPose).setReversed().build()

        intake2 = follower.pathBuilder()
            .buildCurvedTangentLine(poses.farShootPose, poses.farIntake2Control, poses.startIntake2)
            .buildTangentLine(poses.startIntake2, poses.endIntake2).build()

        intake2.setCallbacks(
            ParametricCallback(0, 0.98, follower, {follower.setMaxPower(intakeSpeed)}),
            ParametricCallback(1, 0.98, follower,{follower.setMaxPower(fullSpeed)})
        )

        score2 = follower.pathBuilder()
            .buildTangentLine(poses.endIntake2, poses.farShootPose).setReversed().build()

        park = follower.pathBuilder()
            .buildBasicLine(poses.farShootPose, poses.HPPark).build()

        return listOf(
            Path(scorePreload, fullSpeed),
            Path(intake3, intakeSpeed),
            Path(score3, fullSpeed),
            Path(intake2, intakeSpeed),
            Path(score2, fullSpeed),
            Path(park, fullSpeed)
        )
    }
}
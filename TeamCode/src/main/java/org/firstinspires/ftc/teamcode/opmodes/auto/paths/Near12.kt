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

class Near12 (
    alliance: Alliance
) : BaseAutoPath(
    alliance
) {
    val fullSpeed = 1.0
    val intakeSpeed = 0.67

    lateinit var scorePreload: PathChain
    lateinit var intake1: PathChain
    lateinit var emptyRamp: PathChain
    lateinit var score1: PathChain
    lateinit var intake2: PathChain
    lateinit var score2: PathChain
    lateinit var intake3: PathChain
    lateinit var score3: PathChain
    lateinit var park: PathChain

    override fun buildPaths(follower: Follower): List<Path> {
        scorePreload = follower.pathBuilder()
            .buildBasicLine(poses.nearStartPose, poses.nearShootPoseFacingObelisk).build()

        intake1 = follower.pathBuilder()
            .buildCurvedTangentLine(poses.nearShootPoseFacingObelisk, poses.nearIntake2Control, poses.startIntake2)
            .buildTangentLine(poses.startIntake2, poses.endIntake2).build()

        intake1.setCallbacks(
            ParametricCallback(0, 0.98, follower, {follower.setMaxPower(intakeSpeed)}),
            ParametricCallback(1, 0.98, follower,{follower.setMaxPower(fullSpeed)})
        )

        emptyRamp = follower.pathBuilder()
            .buildCurvedLine(poses.endIntake2, poses.endIntake2Move, poses.emptyRampStart)
            .buildBasicLine(poses.emptyRampStart, poses.emptyRamp).build()

        score1 = follower.pathBuilder()
            .buildBasicLine(poses.emptyRamp, poses.nearShootPose).build()

        intake2 = follower.pathBuilder()
            .buildCurvedTangentLine(poses.nearShootPose, poses.nearIntake1Control, poses.startIntake1)
            .buildTangentLine(poses.startIntake1, poses.endIntake1).build()

        intake2.setCallbacks(
            ParametricCallback(0, 0.98, follower, {follower.setMaxPower(intakeSpeed)}),
            ParametricCallback(1, 0.98, follower,{follower.setMaxPower(fullSpeed)})
        )

        score2 = follower.pathBuilder()
            .buildBasicLine(poses.endIntake1, poses.nearShootPose).build()

        intake3 = follower.pathBuilder()
            .buildCurvedLine(poses.nearShootPose, poses.nearIntake3Control, poses.startIntake3)
            .buildTangentLine(poses.startIntake3, poses.endIntake3).build()

        intake3.setCallbacks(
            ParametricCallback(0, 0.98, follower, {follower.setMaxPower(intakeSpeed)}),
            ParametricCallback(1, 0.98, follower,{follower.setMaxPower(fullSpeed)})
        )

        score3 = follower.pathBuilder()
            .buildTangentLine(poses.endIntake3, poses.endIntake3Move).setReversed()
            .buildBasicLine(poses.endIntake3, poses.nearShootPose).build()

        park = follower.pathBuilder()
            .buildBasicLine(poses.nearShootPose, poses.nearParkPose).build()

        return listOf(
            Path(scorePreload, fullSpeed),
            Path(intake1, fullSpeed),
            Path(emptyRamp, intakeSpeed),
            Path(score1, fullSpeed),
            Path(intake2, fullSpeed),
            Path(score2, fullSpeed),
            Path(intake3, fullSpeed),
            Path(score3, fullSpeed),
            Path(park, fullSpeed)
            )
    }
}
package org.firstinspires.ftc.teamcode.opmodes.auto.paths

import com.pedropathing.follower.Follower
import com.pedropathing.paths.PathChain
import com.pedropathing.paths.callbacks.ParametricCallback
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAutonomous.Companion.PARAMETRIC_END
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.auto.BaseAutoPath
import org.firstinspires.ftc.teamcode.util.auto.Path
import org.firstinspires.ftc.teamcode.util.buildBasicLine
import org.firstinspires.ftc.teamcode.util.buildCurvedLine
import org.firstinspires.ftc.teamcode.util.buildCurvedTangentLine
import org.firstinspires.ftc.teamcode.util.buildIntakeLine
import org.firstinspires.ftc.teamcode.util.buildTangentLine
import org.firstinspires.ftc.teamcode.util.doIntakeSpeed

class Near9 (
    alliance: Alliance
) : BaseAutoPath(
    alliance
) {
    val fullSpeed = 0.8
    val intakeSpeed = 0.5

    lateinit var scorePreload: PathChain
    lateinit var dintake1: PathChain
    lateinit var intake1: PathChain
    lateinit var score1: PathChain
    lateinit var gateempty: PathChain
    lateinit var park: PathChain

    override fun buildPaths(follower: Follower): List<Path> {
        scorePreload = follower.pathBuilder()
            .buildCurvedLine(poses.nearStartPose, poses.nearStartToShoot, poses.nearShootPose).build()

        dintake1 = follower.pathBuilder()
            .buildCurvedLine(poses.nearShootPoseFacingObelisk, poses.farIntake1Control, poses.startIntake1).build()

        intake1 = follower.pathBuilder()
            .buildTangentLine(poses.startIntake1, poses.endIntake1).build()

        score1 = follower.pathBuilder()
            .buildBasicLine(poses.endIntake1, poses.nearShootPose).build()

        gateempty = follower.pathBuilder()
            .buildCurvedLine(poses.nearShootPose, poses.emptyRampControl,poses.emptyRampStart)
            .buildBasicLine(poses.emptyRampStart, poses.emptyRamp).build()

        park = follower.pathBuilder()
            .buildBasicLine(poses.emptyRamp, poses.nearParkPose).build()

        return listOf(
            Path(scorePreload, fullSpeed),
            Path(dintake1, fullSpeed),
            Path(intake1, intakeSpeed),
            Path(score1, fullSpeed),
            Path(gateempty, intakeSpeed),
            Path(park, fullSpeed)
        )
    }
}
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
import org.firstinspires.ftc.teamcode.util.buildTangentLine
import org.firstinspires.ftc.teamcode.util.doIntakeSpeed

class Far12 (alliance: Alliance) : BaseAutoPath(alliance) {
    val fullSpeed = 0.85
    val intakeSpeed = 0.55

    lateinit var scorePreload: PathChain
    lateinit var dintake2: PathChain
    lateinit var intake2: PathChain
    lateinit var demptyRamp: PathChain
    lateinit var emptyRamp: PathChain
    lateinit var score2: PathChain
    lateinit var dintake3: PathChain
    lateinit var intake3: PathChain
    lateinit var score3: PathChain
    lateinit var dHP: PathChain
    lateinit var HP: PathChain
    lateinit var dScoreHP: PathChain
    lateinit var park: PathChain

    override fun buildPaths(follower: Follower): List<Path> {
        scorePreload = follower.pathBuilder()
            .buildBasicLine(poses.farStartPose, poses.farShootPoseFacingObelisk).build()

        dintake2 = follower.pathBuilder()
            .buildCurvedLine(poses.farShootPoseFacingObelisk, poses.farIntake2Control,poses.startIntake2).build()

        intake2 = follower.pathBuilder()
            .buildTangentLine(poses.startIntake2, poses.endIntake2).build()

        demptyRamp = follower.pathBuilder()
            .buildCurvedLine(poses.endIntake2, poses.endIntake2Move, poses.emptyRampStart).build()

        emptyRamp = follower.pathBuilder()
            .buildBasicLine(poses.emptyRampStart, poses.emptyRamp).build()

        score2 = follower.pathBuilder()
            .buildCurvedLine(poses.emptyRamp, poses.emptyRampControl, poses.farShootPose).build()

        dintake3 = follower.pathBuilder()
            .buildCurvedLine(poses.farShootPose,poses.farIntake3Control,poses.startIntake3).build()

        intake3 = follower.pathBuilder()
            .buildTangentLine(poses.startIntake3, poses.endIntake3).build()

        score3 = follower.pathBuilder()
            .buildBasicLine(poses.endIntake3, poses.farShootPose).build()

        dHP = follower.pathBuilder()
            .buildBasicLine(poses.farShootPose,  poses.HPIntakeStart).build()

        HP = follower.pathBuilder()
            .buildTangentLine(poses.HPIntakeStart, poses.HPIntakeEnd).build()

        dScoreHP = follower.pathBuilder()
            .buildBasicLine(poses.HPIntakeEnd, poses.farShootPose).build()

        park = follower.pathBuilder()
            .buildBasicLine(poses.farShootPose, poses.HPPark).build()

        return listOf(
            Path(scorePreload, fullSpeed),
            Path(dintake2, fullSpeed),
            Path(intake2, intakeSpeed),
            Path(demptyRamp, fullSpeed),
            Path(emptyRamp, intakeSpeed),
            Path(score2, fullSpeed),
            Path(dintake3, fullSpeed),
            Path(intake3, intakeSpeed),
            Path(score3, fullSpeed),
            Path(dHP, fullSpeed),
            Path(HP, 0.25),
            Path(dScoreHP, fullSpeed),
            Path(park, fullSpeed)
        )
    }
}
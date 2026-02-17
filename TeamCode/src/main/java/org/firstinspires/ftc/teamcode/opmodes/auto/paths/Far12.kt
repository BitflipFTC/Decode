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

class Far12(alliance: Alliance): BaseAutoPath(alliance) {

    val fullSpeed = 1.0
    val intakeSpeed = 0.67

    lateinit var scorePreload: PathChain
    lateinit var intake2: PathChain
    lateinit var emptyRamp: PathChain
    lateinit var score2: PathChain
    lateinit var intake3: PathChain
    lateinit var score3: PathChain
    lateinit var dHP: PathChain
    lateinit var dScore: PathChain
    lateinit var park: PathChain

    override fun buildPaths(follower: Follower): List<Path> {
        scorePreload = follower.pathBuilder()
            .buildBasicLine(poses.farStartPose, poses.farShootPoseFacingObelisk).build()


        intake2 = follower.pathBuilder()
            .buildCurvedTangentLine(poses.farShootPoseFacingObelisk, poses.farIntake2Control, poses.startIntake2)
            .buildTangentLine(poses.startIntake2, poses.endIntake2).build()

        intake2.doIntakeSpeed(follower, fullSpeed, intakeSpeed)

        emptyRamp = follower.pathBuilder()
            .buildCurvedLine(poses.endIntake2, poses.endIntake2Move, poses.emptyRampStart)
            .buildBasicLine(poses.emptyRampStart, poses.emptyRamp).build()

        score2 = follower.pathBuilder()
            .buildBasicLine(poses.emptyRamp, poses.farShootPose).build()

        intake3 = follower.pathBuilder()
            .buildCurvedTangentLine(poses.farShootPose, poses.farIntake3Control, poses.startIntake3)
            .buildTangentLine(poses.startIntake3, poses.endIntake3).build()

        intake3.doIntakeSpeed(follower, fullSpeed, intakeSpeed)

        score3 = follower.pathBuilder()
            .buildBasicLine(poses.endIntake3, poses.farShootPose).build()

        dHP = follower.pathBuilder()
            .buildTangentLine(poses.farShootPose, poses.HPIntakeStart)
            .buildTangentLine(poses.HPIntakeStart, poses.HPIntakeEnd).build()

        dHP.doIntakeSpeed(follower, fullSpeed, intakeSpeed)

        dScore = follower.pathBuilder()
            .buildTangentLine(poses.HPIntakeEnd, poses.farShootPose).build()

        park = follower.pathBuilder()
            .buildBasicLine(poses.farShootPose, poses.farParkPose).build()

        return listOf(
            Path(scorePreload, fullSpeed),
            Path(intake2, fullSpeed),
            Path(emptyRamp, intakeSpeed),
            Path(score2, fullSpeed),
            Path(intake3, fullSpeed),
            Path(score3, fullSpeed),
            Path(dHP, fullSpeed),
            Path(dScore, fullSpeed),
            Path(park, fullSpeed)
        )
    }
}
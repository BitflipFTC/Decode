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

class Near15(alliance: Alliance) : BaseAutoPath(alliance) {
    val fullSpeed = 1.0
    val intakeSpeed = 0.5

    lateinit var scorePreload: PathChain
    lateinit var dintake2: PathChain
    lateinit var score2: PathChain
    lateinit var dgateintake: PathChain
    lateinit var dogateempty: PathChain
    lateinit var intakegate: PathChain
    lateinit var scoregate: PathChain
    lateinit var dintake1: PathChain
    lateinit var score1: PathChain
    lateinit var dintake3: PathChain
    lateinit var score3: PathChain

    override fun buildPaths(follower: Follower): List<Path> {
        scorePreload = follower.pathBuilder()
            .buildBasicLine(poses.nearStartPose, poses.nearShootPose).build()

        dintake2 = follower.pathBuilder()
            .buildCurvedTangentLine(poses.nearShootPose,poses.nearIntake2Control,poses.endIntake2)
            .build()

        dintake2.doIntakeSpeed(follower,fullSpeed,intakeSpeed)

        score2 = follower.pathBuilder()
            .buildCurvedTangentLine(poses.endIntake2, poses.farIntake2Control, poses.nearShootPose).setReversed().build()

        dgateintake = follower.pathBuilder()
            .buildCurvedLine(poses.nearShootPose.withHeading(Math.toRadians(270.0)), poses.nearIntake2Control, poses.behindGateIntake)
            .build()

        dogateempty = follower.pathBuilder()
            .buildBasicLine(poses.behindGateIntake, poses.gateEmptyForIntake)
            .build()

        intakegate = follower.pathBuilder()
            .buildBasicLine(poses.gateEmptyForIntake, poses.gateIntake)
            .build()

        scoregate = follower.pathBuilder()
            .buildCurvedLine(poses.gateIntake, poses.farIntake2Control, poses.nearShootPose).build()

        dintake1 = follower.pathBuilder()
            .buildCurvedLine(poses.nearShootPose, poses.nearIntake1Control,poses.endIntake1)
            .build()
        dintake1.doIntakeSpeed(follower,fullSpeed,intakeSpeed)

        score1 = follower.pathBuilder()
            .buildTangentLine(poses.endIntake1, poses.nearShootPose).setReversed().build()

        dintake3 = follower.pathBuilder()
            .buildCurvedTangentLine(poses.nearShootPose,poses.farIntake3Control,  poses.endIntake3)
            .build()

        dintake3.doIntakeSpeed(follower,fullSpeed,intakeSpeed)

        score3 = follower.pathBuilder()
            .buildCurvedTangentLine(poses.endIntake3, poses.endIntake3Move, poses.nearShootPoseInZone).setReversed().build()

        return listOf(
            Path(scorePreload, fullSpeed),
            Path(dintake2, fullSpeed),
            Path(score2, fullSpeed),
            Path(dgateintake, fullSpeed),
            Path(dogateempty, fullSpeed),
            Path(intakegate, fullSpeed),
            Path(scoregate, fullSpeed),
            Path(dintake1, fullSpeed),
            Path(score1, fullSpeed),
            Path(dintake3, fullSpeed),
            Path(score3, fullSpeed),
        )
    }
}
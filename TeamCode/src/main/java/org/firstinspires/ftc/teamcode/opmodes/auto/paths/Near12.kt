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

class Near12(alliance: Alliance) : BaseAutoPath(alliance) {
    val fullSpeed = 1.0
    val intakeSpeed = 0.6

    lateinit var scorePreload: PathChain
    lateinit var dintake2: PathChain
    lateinit var score2: PathChain
    lateinit var dgateintake: PathChain
    lateinit var dogateempty: PathChain
    lateinit var intakegate: PathChain
    lateinit var scoregate: PathChain
    lateinit var dintake1: PathChain
    lateinit var score1: PathChain

    override fun buildPaths(follower: Follower): List<Path> {
        scorePreload = follower.pathBuilder()
            .buildCurvedLine(poses.nearStartPose, poses.nearStartToShoot,poses.nearShootPose).build()

        dintake2 = follower.pathBuilder()
            .buildCurvedTangentLine(poses.nearShootPose,poses.nearIntake2Control,poses.endIntake2)
            .build()

        score2 = follower.pathBuilder()
            .buildCurvedTangentLine(poses.endIntake2, poses.farIntake2Control, poses.nearShootPose).setReversed().build()

        dgateintake = follower.pathBuilder()
            .buildCurvedLine(poses.nearShootPose.withHeading(Math.toRadians(270.0)), poses.nearIntake2Control, poses.behindGateIntake)
            .build()

        dogateempty = follower.pathBuilder()
            .buildBasicLine(poses.behindGateIntake, poses.gateEmptyForIntake)
            .build()

        intakegate = follower.pathBuilder()
            .buildCurvedLine(poses.gateEmptyForIntake, poses.behindGateIntake, poses.gateIntake)
            .build()

        scoregate = follower.pathBuilder()
            .buildCurvedLine(poses.gateIntake, poses.farIntake2Control, poses.nearShootPose).build()

        dintake1 = follower.pathBuilder()
            .buildCurvedLine(poses.nearShootPose, poses.nearIntake1Control,poses.endIntake1)
            .build()

        score1 = follower.pathBuilder()
            .buildTangentLine(poses.endIntake1, poses.nearShootPoseInZone).setReversed().build()

        return listOf(
                Path(scorePreload, fullSpeed),
                Path(dintake2, intakeSpeed),
                Path(score2, fullSpeed),
                Path(dgateintake, fullSpeed),
                Path(dogateempty, intakeSpeed),
                Path(intakegate, intakeSpeed),
                Path(scoregate, fullSpeed),
                Path(dintake1, intakeSpeed),
                Path(score1, fullSpeed),
            )
    }
}
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
    val fullSpeed = 0.9
    val intakeSpeed = 0.67
    val gateSpeed = 0.4

    lateinit var scorePreload: PathChain
    lateinit var dintake2: PathChain
    lateinit var intake2: PathChain
    lateinit var score2: PathChain
    lateinit var dgateintake: PathChain
    lateinit var dogateempty: PathChain
    lateinit var intakegate: PathChain
    lateinit var scoregate: PathChain
    lateinit var dintake1: PathChain
    lateinit var intake1: PathChain
    lateinit var score1: PathChain
    lateinit var dintake3: PathChain
    lateinit var intake3: PathChain
    lateinit var score3: PathChain
    lateinit var park: PathChain

    override fun buildPaths(follower: Follower): List<Path> {
        scorePreload = follower.pathBuilder()
            .buildBasicLine(poses.nearStartPose, poses.nearShootPoseFacingObelisk).build()

        dintake2 = follower.pathBuilder()
            .buildCurvedLine(poses.nearShootPose,poses.farIntake1Control,poses.startIntake1).build()

        intake2 = follower.pathBuilder()
            .buildTangentLine(poses.startIntake1, poses.endIntake1).build()

        score2 = follower.pathBuilder()
            .buildTangentLine(poses.endIntake1, poses.nearShootPose).setReversed().build()

        dgateintake = follower.pathBuilder()
            .buildCurvedTangentLine(poses.nearStartPose, poses.farIntake2Control, poses.behindGateIntake).build()

        dogateempty = follower.pathBuilder()
            .buildBasicLine(poses.behindGateIntake, poses.gateEmptyForIntake).build()

        intakegate = follower.pathBuilder()
            .buildBasicLine(poses.gateEmptyForIntake,
                poses.gateIntake).build()

        scoregate = follower.pathBuilder()
            .buildCurvedTangentLine(poses.gateIntake, poses.farIntake2Control, poses.nearShootPose).build()

        dintake1 = follower.pathBuilder()
            .buildCurvedLine(poses.nearShootPoseFacingObelisk, poses.farIntake2Control,poses.startIntake2).build()

        intake1 = follower.pathBuilder()
            .buildTangentLine(poses.startIntake2, poses.endIntake2).build()

        score1 = follower.pathBuilder()
            .buildCurvedLine(poses.emptyRamp, poses.emptyRampControl, poses.nearShootPose).build()

        dintake3 = follower.pathBuilder()
            .buildCurvedTangentLine(poses.nearShootPose,poses.farIntake3Control,  poses.startIntake3).build()

        intake3 = follower.pathBuilder()
            .buildTangentLine(poses.startIntake3, poses.endIntake3).build()

        score3 = follower.pathBuilder()
            .buildCurvedTangentLine(poses.endIntake3, poses.endIntake3Move, poses.nearShootPose).setReversed().build()

        park = follower.pathBuilder()
            .buildBasicLine(poses.nearShootPose, poses.nearParkPose).build()

        return listOf(
            Path(scorePreload, fullSpeed),
            Path(dintake2, fullSpeed),
            Path(intake2, intakeSpeed),
            Path(score2, fullSpeed),
            Path(dgateintake, fullSpeed),
            Path(dogateempty, gateSpeed),
            Path(intakegate, gateSpeed),
            Path(scoregate, fullSpeed),
            Path(dintake1, fullSpeed),
            Path(intake1, intakeSpeed),
            Path(score1, fullSpeed),
            Path(dintake3, fullSpeed),
            Path(intake3, intakeSpeed),
            Path(score3, fullSpeed),
            Path(park, fullSpeed)
        )
    }
}
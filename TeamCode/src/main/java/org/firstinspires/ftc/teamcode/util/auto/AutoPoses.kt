package org.firstinspires.ftc.teamcode.util.auto

import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.teamcode.util.Alliance
import kotlin.math.atan2

class AutoPoses(
    val alliance: Alliance
) {
    private fun p (p: Pose): Pose = if (alliance == Alliance.RED) p else p.mirror()

    private val HORIZONTAL_INTAKE_START = 100.0
    private val HORIZONTAL_INTAKE_END = 136.0
    private val HORIZONTAL_INTAKE_MOVE_BACK = 110.0

    private val INTAKE_1_VERTICAL = 84.0
    private val INTAKE_2_VERTICAL = 58.5
    private val INTAKE_3_VERTICAL = 34.0
    private val INTAKE_HEADING = 0.0

    val farStartPose   = p(Pose(88.0, 9.0, Math.toRadians(90.0)))
    val farShootPoseFacingObelisk = p(Pose(88.0, 18.0, Math.toRadians(90.0)))
    val farShootPose   = p(Pose(88.0, 18.0, Math.toRadians(90.0)))
    val farParkPose    = p(Pose(105.500, 33.500, Math.toRadians(90.0)))
    val nearStartPose  = p(Pose(116.4, 129.6, 0.7463))
    val nearShootPoseFacingObelisk = p(Pose(88.0, 88.0, atan2(144.0 - HORIZONTAL_INTAKE_START, 72.0 - HORIZONTAL_INTAKE_START)))
    val nearShootPose  = p(Pose(88.0, 88.0, INTAKE_HEADING))
    val nearParkPose   = p(Pose(120.0, 70.0, Math.toRadians(270.0)))
    val nearIntake1Control = p(Pose(nearShootPose.x, INTAKE_1_VERTICAL))
    val farIntake1Control = p(Pose(farStartPose.x, INTAKE_1_VERTICAL))
    val startIntake1   = p(Pose(HORIZONTAL_INTAKE_START, INTAKE_1_VERTICAL, INTAKE_HEADING))
    val endIntake1     = p(Pose(128.0, INTAKE_1_VERTICAL, INTAKE_HEADING))
    val nearIntake2Control = p(Pose(nearShootPose.x, INTAKE_2_VERTICAL))
    val farIntake2Control = p(Pose(farStartPose.x, INTAKE_2_VERTICAL))
    val startIntake2   = p(Pose(HORIZONTAL_INTAKE_START, INTAKE_2_VERTICAL, INTAKE_HEADING))
    val endIntake2     = p(Pose(HORIZONTAL_INTAKE_END, INTAKE_2_VERTICAL, INTAKE_HEADING))
    val endIntake2Move = p(Pose(HORIZONTAL_INTAKE_MOVE_BACK, endIntake2.y, INTAKE_HEADING))
    val nearIntake3Control = p(Pose(nearShootPose.x, INTAKE_3_VERTICAL))
    val farIntake3Control = p(Pose(farStartPose.x, INTAKE_3_VERTICAL))
    val startIntake3   = p(Pose(HORIZONTAL_INTAKE_START, INTAKE_3_VERTICAL, INTAKE_HEADING))
    val endIntake3     = p(Pose(HORIZONTAL_INTAKE_END, INTAKE_3_VERTICAL, INTAKE_HEADING))
    val endIntake3Move = p(Pose(HORIZONTAL_INTAKE_MOVE_BACK, endIntake3.y, INTAKE_HEADING))
    val emptyRampStart = p(Pose(120.0, 68.0, Math.toRadians(90.0)))
    val emptyRamp      = p(Pose(129.0, 68.0, Math.toRadians(90.0)))
    val emptyRampControl = p(Pose(nearShootPose.x, INTAKE_2_VERTICAL))
    val redPark        = p(Pose(38.75, 33.25, Math.toRadians(90.0)))
    val HPIntakeEnd    = p(Pose(HORIZONTAL_INTAKE_END, 9.0, Math.toRadians(0.0)))
    val HPIntakeStart = p(Pose(120.0, 9.0, 0.0))
    val HPPark = p(Pose(110.0, 14.0, 0.0))

    val behindGateIntake = p(Pose(124.0, 60.0, Math.toRadians(0.0)))
    val gateEmptyForIntake = p(Pose(130.0, 60.0, Math.toRadians(20.0)))
    val gateIntake = p(Pose(132.0, 53.0, Math.toRadians(35.0)))

    val farShootTeleopOwnGate = p(Pose(88.0, 9.0, 0.0))
    val farShootTeleopHP = p(Pose(88.0, 9.0, Math.PI))
}
package org.firstinspires.ftc.teamcode.util.auto

import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.teamcode.util.Alliance
import kotlin.math.atan2

class AutoPoses(
    val alliance: Alliance
) {
    private fun p (p: Pose): Pose = if (alliance == Alliance.RED) p else p.mirror()

    private val HORIZONTAL_INTAKE_START = 100.0
    private val HORIZONTAL_INTAKE_END = 132.0
    private val HORIZONTAL_INTAKE_MOVE_BACK = 125.0

    private val INTAKE_1_VERTICAL = 84.0
    private val INTAKE_2_VERTICAL = 58.5
    private val INTAKE_3_VERTICAL = 34.0
    private val INTAKE_HEADING = 0.0

    val farStartPose   = p(Pose(88.0, 8.5, Math.toRadians(90.0)))
    val farShootPoseFacingObelisk = p(Pose(88.0, 14.0, Math.toRadians(90.0))) 
    val farShootPose   = p(Pose(88.0, 14.0, Math.toRadians(0.0)))
    val farParkPose    = p(Pose(105.500, 33.500, Math.toRadians(90.0)))
    val nearStartPose  = p(Pose(117.4, 128.9, 0.7188))
    val nearShootPoseFacingObelisk = p(Pose(110.0, 110.0, atan2(144.0 - 110.0, 72.0 - 110.0)))
    val nearShootPose  = p(Pose(110.0, 110.0, INTAKE_HEADING))
    val nearParkPose   = p(Pose(125.0, 90.0, INTAKE_HEADING))
    val nearIntake1Control = p(Pose(HORIZONTAL_INTAKE_START, nearShootPose.y))
    val farIntake1Control = p(Pose(farStartPose.x, INTAKE_1_VERTICAL))
    val startIntake1   = p(Pose(HORIZONTAL_INTAKE_START, INTAKE_1_VERTICAL, INTAKE_HEADING))
    val endIntake1     = p(Pose(129.0, INTAKE_1_VERTICAL, INTAKE_HEADING))
    val nearIntake2Control = p(Pose(HORIZONTAL_INTAKE_START, nearShootPose.y))
    val farIntake2Control = p(Pose(farStartPose.x, INTAKE_2_VERTICAL))
    val startIntake2   = p(Pose(HORIZONTAL_INTAKE_START, INTAKE_2_VERTICAL, INTAKE_HEADING))
    val endIntake2     = p(Pose(HORIZONTAL_INTAKE_END, INTAKE_2_VERTICAL, INTAKE_HEADING))
    val endIntake2Move = p(Pose(HORIZONTAL_INTAKE_MOVE_BACK, endIntake2.y, INTAKE_HEADING))
    val nearIntake3Control = p(Pose(HORIZONTAL_INTAKE_START, nearShootPose.y))
    val farIntake3Control = p(Pose(farStartPose.x, INTAKE_3_VERTICAL))
    val startIntake3   = p(Pose(HORIZONTAL_INTAKE_START, INTAKE_3_VERTICAL, INTAKE_HEADING))
    val endIntake3     = p(Pose(HORIZONTAL_INTAKE_END, INTAKE_3_VERTICAL, INTAKE_HEADING))
    val endIntake3Move = p(Pose(HORIZONTAL_INTAKE_MOVE_BACK, endIntake3.y, INTAKE_HEADING))
    val emptyRampStart = p(Pose(120.0, 77.0, Math.toRadians(90.0)))
    val emptyRamp      = p(Pose(127.5, 77.0, Math.toRadians(90.0)))
    val redPark        = p(Pose(38.75, 33.25, Math.toRadians(90.0)))
    val redHPCorner    = p(Pose(8.4, 7.5, Math.toRadians(90.0)))
}
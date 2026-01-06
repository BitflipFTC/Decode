package org.firstinspires.ftc.teamcode.opmodes.auto

import com.pedropathing.geometry.Pose

private const val horizontalIntakeStart = 100.0
private const val horizontalIntakeEnd = 133.0
private const val horizontalIntakeMoveBack = 125.0

private const val intake1Vertical = 84.0
private const val intake2Vertical = 58.5
private const val intake3Vertical = 34.0
private val intakeHeading = Math.toRadians(0.0)

val farStartPose = Pose(88.0, 8.5, Math.toRadians(90.0))
val farShootPose = Pose(88.0, 14.0, Math.toRadians(0.0))
val farParkPose = Pose(105.500, 33.500, Math.toRadians(90.0))
val nearStartPose = Pose(118.2, 129.5, 0.738)
val nearShootPose = Pose(88.0, 88.0, intakeHeading)
val nearParkPose = Pose(125.0, 90.0, intakeHeading)
val intake1Control = Pose(nearShootPose.x, intake1Vertical)
val startIntake1 = Pose(horizontalIntakeStart, intake1Vertical, intakeHeading)
val endIntake1 = Pose(125.5, intake1Vertical, intakeHeading)
val intake2Control = Pose(nearShootPose.x, intake2Vertical)
val startIntake2 = Pose(horizontalIntakeStart, intake2Vertical, intakeHeading)
val endIntake2 = Pose(horizontalIntakeEnd, intake2Vertical, intakeHeading)
val endIntake2Move = Pose(horizontalIntakeMoveBack, endIntake2.y, intakeHeading)
val intake3Control = Pose(nearShootPose.x, intake3Vertical)
val startIntake3 = Pose(horizontalIntakeStart, intake3Vertical, intakeHeading)
val endIntake3 = Pose(horizontalIntakeEnd, intake3Vertical, intakeHeading)
val endIntake3Move = Pose(horizontalIntakeMoveBack, endIntake3.y, intakeHeading)

val emptyRampStart = Pose(120.0, 77.0, Math.toRadians(90.0))
val emptyRamp = Pose(127.5, 77.0, Math.toRadians(90.0))
package org.firstinspires.ftc.teamcode.opmodes.auto

import org.firstinspires.ftc.teamcode.util.Alliance

abstract class BaseAutoPath(
    val alliance: Alliance
) : AutoPath {
    val poses = AutoPoses(alliance)
}
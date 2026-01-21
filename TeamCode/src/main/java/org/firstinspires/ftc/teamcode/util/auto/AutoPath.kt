package org.firstinspires.ftc.teamcode.util.auto

import com.pedropathing.follower.Follower

interface AutoPath {
    fun buildPaths(follower: Follower): List<Path>
}

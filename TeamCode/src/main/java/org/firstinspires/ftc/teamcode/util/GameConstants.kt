package org.firstinspires.ftc.teamcode.util

object GameConstants {
    var alliance: Alliance = Alliance.NONE
    var motifPattern: MotifPattern = MotifPattern.NONE

    fun reset() {
        alliance = Alliance.NONE
        motifPattern = MotifPattern.NONE
    }
}
package org.firstinspires.ftc.teamcode.util.components

import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.MotifPattern

object GameConstants: Component {
    var alliance: Alliance? = null
    var motifPattern: MotifPattern? = null

    override fun init() {
        alliance = null
        motifPattern = null
    }
}
package org.firstinspires.ftc.teamcode.util

import com.bylazar.utils.LoopTimer
import org.firstinspires.ftc.robotcore.external.Telemetry

object BetterLoopTimeComponent {
    private val loopTimer = LoopTimer(10)
    fun start () {
        loopTimer.start()
    }

    fun update(telemetry: Telemetry) {
        loopTimer.end()
        loopTimer.start()
        telemetry.run {
            addData("Loop ms", loopTimer.ms.toDouble())
            addData("Loop hz", loopTimer.hz)
        }
    }
}
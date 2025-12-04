package org.firstinspires.ftc.teamcode.util

import com.bylazar.utils.LoopTimer
import dev.nextftc.core.components.Component
import dev.nextftc.ftc.ActiveOpMode

object BetterLoopTimeComponent : Component {
    private val loopTimer = LoopTimer(10)
    override fun preStartButtonPressed() {
        loopTimer.start()
    }

    override fun postUpdate() {
        loopTimer.end()
        loopTimer.start()
        ActiveOpMode.telemetry.run {
            addData("Loop ms", loopTimer.ms.toDouble())
            addData("Loop hz", loopTimer.hz)
        }
    }
}
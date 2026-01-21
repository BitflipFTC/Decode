package org.firstinspires.ftc.teamcode.util.components

import com.bylazar.utils.LoopTimer
import com.skeletonarmy.marrow.OpModeManager
import org.firstinspires.ftc.teamcode.util.components.Component

object BetterLoopTimeComponent : Component {
    private val loopTimer = LoopTimer(10)
    override fun init() {
        loopTimer.start()
    }

    override fun periodic() {
        loopTimer.end()
        loopTimer.start()
        OpModeManager.getTelemetry()?.run {
            addData("Loop ms", loopTimer.ms.toDouble())
            addData("Loop hz", loopTimer.hz)
        }
    }
}
package org.firstinspires.ftc.teamcode.util

import com.bylazar.utils.LoopTimer
import dev.nextftc.core.components.Component
import dev.nextftc.ftc.ActiveOpMode

object BetterLoopTimeComponent : Component {
    val loopTimer = LoopTimer(10)

    override fun postStartButtonPressed() = loopTimer.start()
    override fun preUpdate() = update()
    override fun preStop() = loopTimer.end()

    private fun update() {
        ActiveOpMode.telemetry.addData("Loop Hz", "%05.2f", loopTimer.hz)
        ActiveOpMode.telemetry.addData("Loop ms", "%05.2f", loopTimer.ms.toDouble())
    }
}
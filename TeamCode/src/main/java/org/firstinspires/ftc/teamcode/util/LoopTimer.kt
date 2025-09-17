package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.util.ElapsedTime

class LoopTimer() {
    private val timer : ElapsedTime = ElapsedTime()
    private var lastTime : Double = timer.milliseconds()

    fun getLoopTime() : Double {
        return timer.milliseconds() - lastTime
    }

    fun reset() {
        lastTime = timer.milliseconds()
    }
}
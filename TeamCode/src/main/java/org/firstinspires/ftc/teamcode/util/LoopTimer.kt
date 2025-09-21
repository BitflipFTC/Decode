package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.util.ElapsedTime

class LoopTimer() {
    private val timer : ElapsedTime = ElapsedTime()

    fun getLoopTime() : Double {
        val x = timer.milliseconds()
        reset()
        return x
    }

    fun reset() {
        timer.reset()
    }
}
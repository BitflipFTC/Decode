package org.firstinspires.ftc.teamcode.util.components

interface Component {
    fun init()
    fun initLoop() {}
    fun periodic() {}
}
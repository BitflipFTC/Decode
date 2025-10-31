package org.firstinspires.ftc.teamcode.util

fun Boolean.toInt(): Int {
    return if(this) 1 else 0
}

enum class Artifact {
    GREEN,
    PURPLE,
    NONE;

    fun firstLetter() = this.name.first()
}
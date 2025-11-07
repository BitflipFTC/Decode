package org.firstinspires.ftc.teamcode.util

import dev.nextftc.core.commands.Command

class RetryCommand(
    private val command: Command,
    private val completionCondition: () -> Boolean,
    private val retryTimes: Int
): Command() {
    private var timesRetried = 0

    override var isDone = false

    override fun start() {
        addRequirements(command.requirements)

        command.start()
    }

    override fun update() {
        if (!command.isDone) {
            command.update()
            return
        }

        if (!completionCondition.invoke() && timesRetried < retryTimes) {
            command.stop(false)
            command.start()
            timesRetried++
        } else {
            isDone = true
        }
    }

    override fun stop(interrupted: Boolean) {
        command.stop(interrupted)
    }
}
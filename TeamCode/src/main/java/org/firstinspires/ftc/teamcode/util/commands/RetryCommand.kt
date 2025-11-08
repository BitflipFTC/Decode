package org.firstinspires.ftc.teamcode.util.commands

import dev.nextftc.core.commands.Command

/**
 * This command functions like a `RepeatCommand`, except it only repeats IF the `completionCondition` returns false.
 * After its initial run, the command will run again up to `retryTimes` times, OR until the `completionCondition` returns true.
 * @param command the condition to run
 * @param completionCondition a lambada returning a boolean that is evaluated after every command completion.
 * @param retryTimes the amount of times to retry the command if it is initially unsuccessful
 */
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
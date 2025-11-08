/*
 * NextFTC: a user-friendly control library for FIRST Tech Challenge
 *     Copyright (C) 2025 Rowan McAlpin
 *
 *     This program is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *
 *     This program is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public License
 *     along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

package org.firstinspires.ftc.teamcode.util.commands

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.utility.NullCommand

/**
 * This command takes in a command and runs it exactly as many times as specified, unless interrupted.
 * @param command the condition to run
 * @param timesToRunLambada the amount of times to run the command, as a lambada. This allows you to pass in a method reference so you can have a variable number of repeats.
 */
class RepeatCommand (
    private val command: Command,
    private val timesToRunLambada: () -> Int
) : Command() {
    private var timesRun: Int = 0

    var timesToRun = 0

    init {
        requires(command.requirements)
        named("Repeat Command")
        setInterruptible(true)
    }

    override val isDone: Boolean
        get() = command.isDone && timesRun >= timesToRun

    override fun start() {
        timesRun = 0
        timesToRun = timesToRunLambada.invoke()

        command.start()

        if (timesToRun == 0) {
            this.stop(false)
        }

        timesRun++
    }

    override fun update() {
        command.update()

        if (command.isDone && timesRun < timesToRun) {
            command.stop(false)
            command.start()
            timesRun++
        }
    }

    override fun stop(interrupted: Boolean) = command.stop(interrupted)
}
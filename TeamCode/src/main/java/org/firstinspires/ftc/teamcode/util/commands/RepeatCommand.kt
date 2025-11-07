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

class RepeatCommand (
    private val command: () -> Command,
    private val timesToRunLambada: () -> Int
) : Command() {
    private var timesRun: Int = 0
    private var mCommand: Command = NullCommand()

    var timesToRun = 0

    init {
        requires(command.invoke().requirements)
        named("Repeat Command")
        setInterruptible(true)
    }

    override val isDone: Boolean
        get() = mCommand.isDone && timesRun >= timesToRun

    override fun start() {
        mCommand = command.invoke()
        timesRun = 0
        timesToRun = timesToRunLambada.invoke()

        mCommand.start()

        if (timesToRun == 0) {
            this.stop(false)
        }

        timesRun++
    }

    override fun update() {
        mCommand.update()

        if (mCommand.isDone && timesRun < timesToRun) {
            mCommand.stop(false)
            mCommand = command.invoke()
            mCommand.start()
            timesRun++
        }
    }

    override fun stop(interrupted: Boolean) = mCommand.stop(interrupted)
}
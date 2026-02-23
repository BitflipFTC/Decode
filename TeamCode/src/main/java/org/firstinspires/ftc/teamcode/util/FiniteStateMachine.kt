package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.opmodes.teleop.CombinedTeleOp
import org.firstinspires.ftc.teamcode.util.auto.Path

class FiniteStateMachine (vararg inputStates: State) {

    var currentState: Int = 0
    val states: MutableList<State> = mutableListOf()

    init {
        inputStates.forEach { states.add(it) }
    }

    fun addState (state: State): FiniteStateMachine {
        states.add(state)
        return this
    }

    fun clearStates() = states.clear()

    fun run() {
        if (states.isEmpty() || currentState >= states.size) return

        val activeState = states[currentState]
        if (!activeState.started) {
            activeState.initialize()
            activeState.started = true
        } else {
            activeState.run()
        }
        if (activeState.endCondition()) {
            currentState++
        }
    }
}

open class State(
    val name: String
) {
    open val endCondition: () -> Boolean = { false }
    open val initialize: () -> Unit = {}
    open val run: () -> Unit = {}
    var started = false
}

class InstantState (
    name: String, override val initialize: () -> Unit
) : State(
    name
) {
    override val endCondition = { true }
}

open class InitializeState (
    name: String, override val endCondition: () -> Boolean, override val initialize: () -> Unit
) : State (name)

class FollowPathState (
    name: String, path: Path
) : InitializeState (
    name, { !CombinedTeleOp.follower!!.isBusy }, { CombinedTeleOp.follower!!.followCustomPath(path) }
)

class WaitState (
    millis: Double, name: String = ""
) : State (name) {
    private val timer = ElapsedTime()
    override val initialize = { timer.reset() }
    override val endCondition = { timer.milliseconds() > millis }
}

class WaitUntilState (
    override val endCondition: () -> Boolean
) : State("")

package org.firstinspires.ftc.teamcode.util

class FiniteStateMachine {
    var currentState: Int = 0
    val states: MutableList<State> = mutableListOf()

    fun addState (name: String, startCondition: () -> Boolean = { true }, run: () -> Unit = {},): FiniteStateMachine {
        states.add(State(name, startCondition, run))
        return this
    }

    fun clearStates() = states.clear()
    fun run() {
        if (states.isEmpty() || currentState >= states.size) return

        val activeState = states[currentState]

        if (activeState.startCondition()) {
            activeState.run()
            currentState++
        }
    }
}

class State (
    val name: String,
    val startCondition: () -> Boolean,
    val run: () -> Unit
    )

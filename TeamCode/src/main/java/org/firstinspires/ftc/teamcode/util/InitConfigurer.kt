package org.firstinspires.ftc.teamcode.util

import dev.nextftc.core.components.Component
import dev.nextftc.ftc.ActiveOpMode

object InitConfigurer : Component {
    private val telemetry = ActiveOpMode.telemetry
    private val gamepad1 = ActiveOpMode.gamepad1

    private var hasSelectedAlliance = false
    private var hasSelectedStartingPosition = false

    var selectedAlliance: Alliance = Alliance.NONE
        private set
    var selectedStartingPosition: StartingPosition = StartingPosition.NONE
        private set

    enum class Phase {
        SELECTING_ALLIANCE,
        SELECTING_STARTING_POSITION,
        NONE
    }

    private var phase = Phase.NONE

    override fun postWaitForStart() {
        when (phase) {
            Phase.SELECTING_ALLIANCE          -> {
                telemetry.run {
                    addLine("Selecting alliance")
                    addLine()
                    addLine("Press CROSS  for BLUE")
                    addLine("Press CIRCLE for RED")
                    addLine()
                    addData("Current alliance", selectedAlliance.name)
                }

                selectedAlliance = if (gamepad1.crossWasPressed()) Alliance.BLUE else if (gamepad1.circleWasPressed()) Alliance.RED else selectedAlliance

                if (gamepad1.touchpadWasPressed()) {
                    hasSelectedAlliance = true
                    toNextPhase()
                }
            }

            Phase.SELECTING_STARTING_POSITION -> {
                telemetry.run {
                    addLine("Selecting starting position")
                    addLine()
                    addLine("Press TRIANGLE for GOAL")
                    addLine("Press SQUARE   for FAR")
                    addLine()
                    addData("Current starting position", selectedStartingPosition.name)
                }

                selectedStartingPosition = if (gamepad1.triangleWasPressed()) StartingPosition.GOAL else if (gamepad1.squareWasPressed()) StartingPosition.FAR else selectedStartingPosition

                if (gamepad1.touchpadWasPressed()) {
                    hasSelectedStartingPosition = true
                    toNextPhase()
                }
            }

            Phase.NONE                        -> toNextPhase()
        }
    }

    fun toNextPhase() {
        // if all the phases are complete, end
        if (hasSelectedAlliance && hasSelectedStartingPosition) return

        phase = if (!hasSelectedAlliance) Phase.SELECTING_ALLIANCE
            else Phase.SELECTING_STARTING_POSITION
    }
}
package org.firstinspires.ftc.teamcode.util

import dev.nextftc.core.components.Component
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.robotcore.external.Telemetry

object InitConfigurer : Component {
    private val telemetry: Telemetry
        get() = ActiveOpMode.telemetry
    private val gamepad1: com.qualcomm.robotcore.hardware.Gamepad
        get() = ActiveOpMode.gamepad1

    private var hasSelectedAlliance = false
    private var hasSelectedStartingPosition = false

    var selectedAlliance: Alliance? = null
        private set
    var selectedStartingPosition: StartingPosition? = null
        private set

    enum class Phase {
        SELECTING_ALLIANCE,
        SELECTING_STARTING_POSITION
    }

    private var phase: Phase? = null

    override fun preInit() {
        phase = null
        selectedAlliance = null
        selectedStartingPosition = null
        hasSelectedAlliance = false
        hasSelectedStartingPosition = false
    }

    override fun postWaitForStart() {
        when (phase) {
            Phase.SELECTING_ALLIANCE          -> {
                telemetry.run {
                    addLine("Selecting alliance")
                    addLine()
                    addLine("Press CROSS  for BLUE")
                    addLine("Press CIRCLE for RED")
                    addLine("Press TOUCHPAD to confirm")
                    addLine()
                    addData("Current alliance", selectedAlliance?.name ?: "NONE SELECTED")
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
                    addLine("Press TOUCHPAD to confirm")
                    addLine()
                    addData("Current starting position", selectedStartingPosition?.name ?: "NONE SELECTED")
                }

                selectedStartingPosition = if (gamepad1.triangleWasPressed()) StartingPosition.GOAL else if (gamepad1.squareWasPressed()) StartingPosition.FAR else selectedStartingPosition

                if (gamepad1.touchpadWasPressed()) {
                    hasSelectedStartingPosition = true
                    toNextPhase()
                }
            }

            null                        -> toNextPhase()
        }
    }

    fun toNextPhase() {
        // if all the phases are complete, end
        if (hasSelectedAlliance && hasSelectedStartingPosition) return

        phase = if (!hasSelectedAlliance) Phase.SELECTING_ALLIANCE
            else Phase.SELECTING_STARTING_POSITION
    }
}
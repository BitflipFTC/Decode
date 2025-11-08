package org.firstinspires.ftc.teamcode.util

import dev.nextftc.core.components.Component
import dev.nextftc.ftc.ActiveOpMode

object TelemetryComponent: Component {
    override fun postWaitForStart() {
        ActiveOpMode.telemetry.update()
    }
    override fun postUpdate() {
        ActiveOpMode.telemetry.update()
    }
}
package org.firstinspires.ftc.teamcode.util

import org.firstinspires.ftc.robotcore.external.Telemetry

object InitConfigurer : Component {
    private val telemetry: Telemetry
        get() = ActiveOpMode.telemetry
    private val gamepad1: com.qualcomm.robotcore.hardware.Gamepad
        get() = ActiveOpMode.gamepad1

    var hasSelectedAlliance = false

    var selectedAlliance: Alliance? = null

    override fun preInit() {
        selectedAlliance = null
        hasSelectedAlliance = false
    }

    override fun postWaitForStart() {
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
        }

    }
}
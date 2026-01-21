package org.firstinspires.ftc.teamcode.util.components

import com.qualcomm.robotcore.hardware.Gamepad
import com.skeletonarmy.marrow.OpModeManager
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.util.Alliance

object InitConfigurer : Component {
    private val telemetry: Telemetry
        get() = OpModeManager.getTelemetry()!!
    private val gamepad1: Gamepad
        get() = OpModeManager.getActiveOpMode()!!.gamepad1

    var hasSelectedAlliance = false

    var selectedAlliance: Alliance? = null

    override fun init() {
        selectedAlliance = null
        hasSelectedAlliance = false
    }

    override fun initLoop() {
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
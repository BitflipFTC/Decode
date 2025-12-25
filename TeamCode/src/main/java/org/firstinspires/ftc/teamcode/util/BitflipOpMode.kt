package org.firstinspires.ftc.teamcode.util

import android.provider.Settings
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

abstract class BitflipOpMode: LinearOpMode() {
    final override fun runOpMode() {
        try {
            OpModeConstants.hardwareMap = this.hardwareMap
            telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
            OpModeConstants.telemetry = this.telemetry

            onInit()

            while (opModeInInit()) {
                onWaitForStart()
                telemetry.update()
            }

            if (!isStopRequested) {
                onStart()

                while(opModeIsActive()) {
                    onUpdate()
                    telemetry.update()
                }
            }

            onStop()

        } catch (e: Exception) {
            // Rethrow the exception as a RuntimeException with the original stack trace at the top
            val runtimeException = RuntimeException(e.message)
            runtimeException.stackTrace = e.stackTrace  // Set the original stack trace at the top
            throw runtimeException  // Throw the custom RuntimeException
        }
    }

    open fun onInit() {}
    open fun onWaitForStart() {}
    open fun onStart() {}
    open fun onUpdate() {}
    open fun onStop() {}
}
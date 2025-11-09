package org.firstinspires.ftc.teamcode.util

import dev.nextftc.core.commands.CommandManager
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.NextFTCOpMode
import java.lang.RuntimeException

open class BitflipOpMode: NextFTCOpMode() {
    override fun runOpMode() {
        try {
            components.forEach { it.preInit() }
            onInit()
            components.reversed().forEach { it.postInit() }

            // Wait for start
            while (opModeInInit()) {
//                components.forEach { it.preWaitForStart() }
                onWaitForStart()
                telemetry.update()
//                components.reversed().forEach { it.postWaitForStart() }
            }

            // If we pressed stop after init (instead of start) we want to skip the rest of the OpMode
            // and jump straight to the end
            if (!isStopRequested) {
                components.forEach { it.preStartButtonPressed() }
                onStartButtonPressed()
                components.reversed().forEach { it.postStartButtonPressed() }

                while (opModeIsActive()) {
                    components.forEach { it.preUpdate() }
                    CommandManager.run()
                    onUpdate()
                    components.reversed().forEach { it.postUpdate() }
                    telemetry.update()
                }
            }

            components.forEach { it.preStop() }
            onStop()
            components.forEach { it.postStop() }
        } catch (e: Exception) {
            // Rethrow the exception as a RuntimeException with the original stack trace at the top
            val runtimeException = RuntimeException(e.message)
            runtimeException.stackTrace = e.stackTrace  // Set the original stack trace at the top
            throw runtimeException  // Throw the custom RuntimeException
        }
    }
}
package org.firstinspires.ftc.teamcode.util

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.ftc.components.LoopTimeComponent
import org.firstinspires.ftc.teamcode.subsystems.ArtifactColorSensor
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.OV9281
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Spindexer
import org.firstinspires.ftc.teamcode.subsystems.Transfer
import org.firstinspires.ftc.teamcode.subsystems.Turret
import org.firstinspires.ftc.teamcode.util.commands.RepeatCommand
import org.firstinspires.ftc.teamcode.util.commands.RetryCommand
import java.lang.RuntimeException
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds

/**
 * Automatically updates telemetry in init and in loop
 * Available subsystems to add in addComponents:
 * - drivetrain
 * - intake
 * - camera
 * - shooter
 * - spindexer
 * - transfer
 * - turret
 * - colorSensor
 */
open class BitflipOpMode: NextFTCOpMode() {
    protected val drivetrain = Drivetrain()
    protected val intake = Intake()
    protected val camera = OV9281()
    protected val shooter = Shooter()
    protected val spindexer = Spindexer()
    protected val transfer = Transfer()
    protected val turret = Turret()
    protected val colorSensor = ArtifactColorSensor()

    init {
        addComponents(
            BulkReadComponent,
            BindingsComponent,
            LoopTimeComponent(),
        )
    }

    override fun runOpMode() {
        try {
            telemetry = JoinedTelemetry(telemetry, PanelsTelemetry.ftcTelemetry)

            components.forEach { it.preInit() }
            onInit()
            components.reversed().forEach { it.postInit() }

            // Wait for start
            while (opModeInInit()) {
//                components.forEach { it.preWaitForStart() }
                BindingsComponent.preWaitForStart()
                onWaitForStart()
                if (components.contains(InitConfigurer)) InitConfigurer.postWaitForStart()
                BulkReadComponent.postWaitForStart()

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

    fun retryShoot() = RetryCommand(
        transfer.shootArtifact(),
        { !shooter.atSetPoint() },
        3
    ).then(InstantCommand { if (!shooter.atSetPoint()) {spindexer.recordOuttake()}})

    fun shootAllArtifacts(delay: Duration) = RepeatCommand(
        SequentialGroup(
            spindexer.tryMotifOuttake(),
            WaitUntil(shooter::atSetPoint),
            retryShoot().thenWait(delay)
        ), spindexer::totalFullSlots
    ).then(spindexer.goToFirstEmptyIntake())
}
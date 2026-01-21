package org.firstinspires.ftc.teamcode.subsystems

import android.util.Log
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.skeletonarmy.marrow.OpModeManager
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.util.Artifact

class ArtifactColorSensor(): SubsystemBase() {
    private val colorSensor: RevColorSensorV3 = OpModeManager.getHardwareMap().get(RevColorSensorV3::class.java, "colorSensor")
    var colors: NormalizedRGBA = NormalizedRGBA()
        private set

    var detectedArtifact: Artifact? = null
        private set
    // make sure this starts above 5.0 to avoid the color sensor making a fake read on the first loop
    var distance: Double = 10.0
    val red: Double
        get() = colors.red.toDouble()
    val green: Double
        get() = colors.green.toDouble()
    val blue: Double
        get() = colors.blue.toDouble()

    var debugTelemetry = true

    init {
        colorSensor.gain = 20.0f
    }

    override fun periodic() {
        // alternate reads to improve loop times
        colors = colorSensor.normalizedColors
        distance = colorSensor.getDistance(DistanceUnit.CM)

        detectedArtifact = if (distance < 3.0) {
            if ((red < blue) && (red < green)) {
                Log.d("FSM", "colors, b: $blue, g: $green, d: $distance")
                if ((blue > green)) {
                    Artifact.PURPLE
                } else {
                    Artifact.GREEN
                }
            } else null
        } else {
            null
        }

        if (debugTelemetry) {
            OpModeManager.getTelemetry()?.run {
                addData("Detected Artifact", detectedArtifact?.name ?: "none")
                addData("Distance", "%05.2fcm", distance)
                addData("green","%07.4f", green)
                addData("red","%07.4f", red)
                addData("blue","%07.4f", blue)
                addLine("------------------------------")
            }
        }
    }

}
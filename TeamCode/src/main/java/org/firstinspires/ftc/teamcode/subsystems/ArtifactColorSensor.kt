package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.util.Artifact

class ArtifactColorSensor: Subsystem {
    private lateinit var colorSensor: RevColorSensorV3
    var colors: NormalizedRGBA = NormalizedRGBA()
        private set

    var detectedArtifact = Artifact.NONE
        private set
    val distance: Double
        get() = colorSensor.getDistance(DistanceUnit.CM)
    val red: Double
        get() = colors.red / colors.alpha.toDouble()
    val green: Double
        get() = colors.green / colors.alpha.toDouble()
    val blue: Double
        get() = colors.blue / colors.alpha.toDouble()

    override fun initialize() {
        colorSensor = ActiveOpMode.hardwareMap.get(RevColorSensorV3::class.java, "colorSensor")
    }

    override fun periodic() {
        colors = colorSensor.normalizedColors

        detectedArtifact = if (distance <= 5.0) {
            if (blue > green) {
                Artifact.PURPLE
            } else {
                Artifact.GREEN
            }
        } else {
            Artifact.NONE
        }

        ActiveOpMode.telemetry.run {
            addData("R", red)
            addData("G", green)
            addData("B", blue)
            addLine()
            addData("Detected Artifact", detectedArtifact.name)
            addData("Detected Artifact Ordinal (graphing)", detectedArtifact.ordinal)
            addData("Light detected", colorSensor.lightDetected)
            addData("Distance", "%05.2fcm", distance)
            addLine("------------------------------")
        }
    }

}
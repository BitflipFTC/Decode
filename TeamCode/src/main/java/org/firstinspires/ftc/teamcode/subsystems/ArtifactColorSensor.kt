package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.util.Artifact

class ArtifactColorSensor: Subsystem {
    private lateinit var colorSensor: ColorRangeSensor
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
        colorSensor = ActiveOpMode.hardwareMap.get(ColorRangeSensor::class.java, "colorSensor")
    }

    override fun periodic() {
        colors = colorSensor.normalizedColors
        if (distance <= 5.0) {
            detectedArtifact =
                if        (red <= 0.02 && green <= 0.02  && blue >= 0.023) {
                    Artifact.PURPLE
                } else if (red <= 0.02 && green >= 0.023 && blue <= 0.022) {
                    Artifact.GREEN
                } else {
                    Artifact.NONE
                }
        }

        ActiveOpMode.telemetry.run {
            addData("Normalized R", red)
            addData("Normalized G", green)
            addData("Normalized B", blue)
            addData("R", colors.red)
            addData("G", colors.green)
            addData("B", colors.blue)
            addLine()
            addData("Detected Artifact", detectedArtifact.name)
            addData("Light detected", colorSensor.lightDetected)
            addData("Distance", "%05.2fcm", distance)
            addLine("------------------------------")
        }
    }

}
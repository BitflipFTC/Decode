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

    override fun initialize() {
        colorSensor = ActiveOpMode.hardwareMap.get(ColorRangeSensor::class.java, "colorSensor")
    }

    override fun periodic() {
        colors = colorSensor.normalizedColors
        if (colorSensor.getDistance(DistanceUnit.CM) <= 5.0) {
            detectedArtifact =
                if (colors.red / colors.alpha <= 0.02 && colors.green / colors.alpha <= 0.02 && colors.blue / colors.alpha >= 0.023) {
                    Artifact.PURPLE
                } else if (colors.red / colors.alpha <= 0.02 && colors.green / colors.alpha >= 0.023 && colors.blue / colors.alpha <= 0.022) {
                    Artifact.GREEN
                } else {
                    Artifact.NONE
                }
        }

        ActiveOpMode.telemetry.run {
            addData("Normalized R", colors.red / colors.alpha)
            addData("Normalized G", colors.green / colors.alpha)
            addData("Normalized B", colors.blue / colors.alpha)
            addData("R", colors.red)
            addData("G", colors.green)
            addData("B", colors.blue)
            addLine()
            addData("Detected Artifact", detectedArtifact.name)
            addData("Light detected", colorSensor.lightDetected)
            addData("Distance", "%05.2fcm", colorSensor.getDistance(DistanceUnit.CM))
            addLine("------------------------------")
        }
    }

}
package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.util.Artifact

class ArtifactColorSensor(): Subsystem {
    private lateinit var colorSensor: RevColorSensorV3
    var colors: NormalizedRGBA = NormalizedRGBA()
        private set

    var detectedArtifact = Artifact.NONE
        private set
    var distance: Double = 0.0
    val red: Double
        get() = colors.red.toDouble()
    val green: Double
        get() = colors.green.toDouble()
    val blue: Double
        get() = colors.blue.toDouble()

    // reading the distance is one i2c read,
    // reading the colors is another.
    // We split these reads into two loops, only reading one per loop,
    // which decreases loop times.
    // when this is false, the colors are read.
    private var readDistance = true

    override fun initialize() {
        colorSensor = ActiveOpMode.hardwareMap.get(RevColorSensorV3::class.java, "colorSensor")
    }

    override fun periodic() {
        // alternate reads to improve loop times
        if (!readDistance) {
            colors = colorSensor.normalizedColors
        } else {
            distance = colorSensor.getDistance(DistanceUnit.CM)
        }
        readDistance = !readDistance

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
            addData("Detected Artifact", detectedArtifact.name)
            addData("Detected Artifact Ordinal (graphing)", detectedArtifact.ordinal)
            addData("Distance", "%05.2fcm", distance)
            addLine("------------------------------")
        }
    }

}
package org.firstinspires.ftc.teamcode.subsystems

import android.util.Log
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
        if (distance <= 3.0) {
            readDistance = !readDistance
            if (!readDistance) {
                colors = colorSensor.normalizedColors
            }

            Log.d("FSM", "colors, b: $blue, g: $green")
            detectedArtifact = if (blue > green) {
                Artifact.PURPLE
            } else {
                Artifact.GREEN
            }
        } else {
            detectedArtifact = null
        }

        if (readDistance) {
            distance = colorSensor.getDistance(DistanceUnit.CM)
        }

        if (debugTelemetry) {
            ActiveOpMode.telemetry.run {
                addData("Detected Artifact", detectedArtifact?.name ?: "none")
                addData("Detected Artifact Ordinal (graphing)", detectedArtifact?.ordinal ?: 2)
                addData("Distance", "%05.2fcm", distance)
                addData("Read distance", readDistance)
                addLine("------------------------------")
            }
        }
    }

}
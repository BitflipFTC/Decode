package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.util.Artifact
import org.firstinspires.ftc.teamcode.util.OpModeConstants.hardwareMap
import org.firstinspires.ftc.teamcode.util.OpModeConstants.telemetry

class ArtifactColorSensor(): Subsystem {
    private var colorSensor: RevColorSensorV3 =
        hardwareMap!!.get(RevColorSensorV3::class.java, "colorSensor")
    var colors: NormalizedRGBA = NormalizedRGBA()
        private set

    var detectedArtifact: Artifact? = null
        private set
    var distance: Double = 0.0
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

    override fun periodic() {
        // alternate reads to improve loop times
        if (distance <= 5.0) {
            readDistance = !readDistance
            if (!readDistance) {
                colors = colorSensor.normalizedColors
            }

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
            telemetry!!.run {
                addData("Detected Artifact", detectedArtifact?.name ?: "none")
                addData("Detected Artifact Ordinal (graphing)", detectedArtifact?.ordinal ?: 2)
                addData("Distance", "%05.2fcm", distance)
                addLine("------------------------------")
            }
        }
    }

}
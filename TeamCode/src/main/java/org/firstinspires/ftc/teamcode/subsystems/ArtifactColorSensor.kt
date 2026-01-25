package org.firstinspires.ftc.teamcode.subsystems

import android.graphics.Color
import android.os.Build
import android.util.Log
import androidx.annotation.RequiresApi
import com.bylazar.configurables.annotations.Sorter
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

    // make sure this starts above 5.0 to avoid the color sensor making a fake read on the first loop
    var distance: Double = 10.0
    val red: Double
        get() = colors.red.toDouble()
    val green: Double
        get() = colors.green.toDouble()
    val blue: Double
        get() = colors.blue.toDouble()

    class HSV() {
        val col = floatArrayOf(0f,0f,0f)
        val h: Float
            get() = col[0]
        val s: Float
            get() = col[1]
        val v: Float
            get() = col[2]
    }

    val hsv = HSV()

    var debugTelemetry = true

    override fun initialize() {
        colorSensor = ActiveOpMode.hardwareMap.get(RevColorSensorV3::class.java, "colorSensor")
        colorSensor.gain = 20.0f
    }

    @RequiresApi(Build.VERSION_CODES.O)
    override fun periodic() {
        // alternate reads to improve loop times
        colors = colorSensor.normalizedColors
        distance = colorSensor.getDistance(DistanceUnit.CM)

        val color = Color.rgb(colors.red * 255, colors.green * 255, colors.blue * 255)
        Color.colorToHSV(color, hsv.col)

        detectedArtifact = if (distance < 3.0) {
            Log.d("FSM", "colors, h: ${hsv.h}, s: ${hsv.s}, v: ${hsv.v}")
            when (hsv.h) {
                in 150.0..185.0     -> {
                    Artifact.GREEN
                }
                in 200.0..250.0 -> {
                    Artifact.PURPLE
                }
                else            -> {
                    null
                }
            }
        } else {
            null
        }

        if (debugTelemetry) {
            ActiveOpMode.telemetry.run {
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
package org.firstinspires.ftc.teamcode.subsystems

import android.graphics.Color
import android.util.Log
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.skeletonarmy.marrow.OpModeManager
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.util.Artifact

class ColorSensor(): SubsystemBase() {
    private val colorSensor: RevColorSensorV3 = OpModeManager.getHardwareMap().get(RevColorSensorV3::class.java, "colorSensor")
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

    init {
        colorSensor.gain = 20.0f
    }

    override fun periodic() {
        // alternate reads to improve loop times
        colors = colorSensor.normalizedColors
        distance = colorSensor.getDistance(DistanceUnit.CM)

        Color.RGBToHSV((colors.red * 255).toInt(), (colors.green * 255).toInt(), (colors.blue * 255).toInt(), hsv.col)

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
            OpModeManager.getTelemetry()?.run {
                addData("Detected Artifact", detectedArtifact?.name ?: "none")
                addData("Distance", "%05.2fcm", distance)
                addData("hue","%07.4f", hsv.h)
                addData("sat","%07.4f", hsv.s)
                addData("val","%07.4f", hsv.v)
                addLine("------------------------------")
            }
        }
    }

}
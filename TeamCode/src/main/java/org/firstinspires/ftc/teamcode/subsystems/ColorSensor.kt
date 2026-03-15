package org.firstinspires.ftc.teamcode.subsystems

import android.graphics.Color
import android.util.Log
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.I2cDeviceSynch
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import com.qualcomm.robotcore.util.Range
import com.qualcomm.robotcore.util.TypeConversion
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.util.Artifact
import java.nio.ByteOrder
import kotlin.math.min
import kotlin.math.pow

class ColorSensor(): Subsystem {
    private lateinit var colorSensor: RevColorSensorV3
    var colors: NormalizedRGBA = NormalizedRGBA()
        private set

    var detectedArtifact: Artifact? = null

    // make sure this starts above 5.0 to avoid the color sensor making a fake read on the first loop
    var distance: Double = 10.0

    var red: Int = 0
    var green: Int = 0
    var blue: Int = 0

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

    val aParam = 325.961
    val binvParam = -0.75934
    val cParam = 26.980
    val maxDist = 6.0 // inches
    fun inFromOptical(rawOptical: Int): Double {
        // can't have a negative number raised to a fractional power. In this situation need to
        // return max value
        if (rawOptical <= cParam) return maxDist

        // compute the distance based on an inverse power law, i.e.
        //  distance = ((RawOptical - c)/a)^(1/b)
        val dist: Double = ((rawOptical - cParam) / aParam).pow(binvParam)

        // distance values change very rapidly with small values of RawOptical and become
        // impractical to fit to a distribution. Returns are capped to an experimentally determined
        // max value.
        return min(dist, maxDist)
    }

    override fun initialize() {
        colorSensor = ActiveOpMode.hardwareMap.get(RevColorSensorV3::class.java, "colorSensor")
        colorSensor.gain = 20.0f
    }

    var preTime: Long = 0
    var postTime: Long = 0

    val limit = 1048575
    override fun periodic() {
        preTime = System.nanoTime()
        val bytes = (colorSensor as I2cDeviceSynch).read(0x08, 14)
        // 0x07 is MAIN_STATUS
        // default value is 0x20, or 00100000
        // b7 b6 are 0, b3 is LS (light sensor) data status, b0 is ps (prox sensor) status
        // ngl read the docs though https://docs.broadcom.com/doc/APDS-9151-DS

        // Read red, green and blue values
        green = (((bytes[7].toInt() and 0x0F) shl 16) or
                ((bytes[6].toInt() and 0xFF) shl 8) or
                (bytes[5].toInt() and 0xFF))

        blue = Range.clip(
            ((((bytes[10].toInt() and 0x0F) shl 16) or
                    ((bytes[9].toInt() and 0xFF) shl 8) or
                    (bytes[8].toInt() and 0xFF)) * 1.55).toInt(),
            0,limit
        )

        red = Range.clip(
            ((((bytes[13].toInt() and 0x0F) shl 16) or
                    ((bytes[12].toInt() and 0xFF) shl 8) or
                    (bytes[11].toInt() and 0xFF)) * 1.07).toInt(),
            0, limit
        )

        // normalize to [0, 1]
        this.colors.red = Range.clip(
            (red.toFloat() * colorSensor.gain) / limit,
            0f,
            1f
        )
        this.colors.green = Range.clip(
            (green.toFloat() * colorSensor.gain) / limit,
            0f,
            1f
        )
        this.colors.blue = Range.clip(
            (blue.toFloat() * colorSensor.gain) / limit,
            0f,
            1f
        )

        val rawOptical = (((bytes[1].toInt() and 0xFF) shl 8) or (bytes[0].toInt() and 0xFF)) and 0x7FF
        distance = DistanceUnit.CM.fromUnit(DistanceUnit.INCH, inFromOptical(rawOptical))

        Color.RGBToHSV((colors.red * 255).toInt(), (colors.green * 255).toInt(), (colors.blue * 255).toInt(), hsv.col)
        postTime = System.nanoTime()



        // old version

//        distance = colorSensor.getDistance(DistanceUnit.CM)
//        colors = colorSensor.normalizedColors

        // end old version

        detectedArtifact = if (distance < 3.0) {
            Log.d("CS", "colors, h: ${hsv.h}, s: ${hsv.s}, v: ${hsv.v}")
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
                addData("Color Sensor Detected Artifact", detectedArtifact?.name ?: "none")
//                addData("Distance", "%05.2fcm", distance)
//                addData("hue","%07.4f", hsv.h)
//                addData("sat","%07.4f", hsv.s)
//                addData("val","%07.4f", hsv.v)
                addData("Read time", "%04.2fms",(postTime - preTime) / 1E6)
                addLine("------------------------------")
            }
        }

    }
}
package org.firstinspires.ftc.teamcode.subsystems

import android.graphics.Color
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import com.qualcomm.robotcore.util.Range
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.util.Artifact
import kotlin.math.min
import kotlin.math.pow

class ColorSensor(): Subsystem {
    // The normal color sensor
    private lateinit var colorSensor: RevColorSensorV3

    // This is the i2c component of the color sensor that we'll use to read the data
    private lateinit var device: I2cDeviceSynchSimple

    var detectedArtifact: Artifact? = null

    // These are the two data sources we read from the color sensor.
    var colors: NormalizedRGBA = NormalizedRGBA()
        private set
    var distance: Double = 0.0
        private set

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

    // This is a copy of the same method found in RevColorSensorV3.java, credits to the author
    private val aParam = 325.961
    private val binvParam = -0.75934
    private val cParam = 26.980
    private val maxDist = 6.0 // inches
    private fun inFromOptical(rawOptical: Int): Double {
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

        // Basically, this gets the field deviceClient from the far removed ancestor of RevColorSensorV3.java (I think it's I2cDeviceSimple.java)
        // then makes it accessible here using reflection, then finally gets that field from our color sensor in order to have access to its i2c register.
        val field = colorSensor.javaClass.superclass.superclass.superclass.getDeclaredField("deviceClient")
        field.isAccessible = true
        device = field.get(colorSensor) as I2cDeviceSynchSimple

        colorSensor.gain = 20.0f

    }

    // Profiling
    var preTime: Long = 0
    var posti2cTime: Long = 0
    var postTime: Long = 0

    // this is 2^20 - 1, the highest possible number in a 20 bit int
    // This is used to normalize the colors
    val limit = 1048575
    override fun periodic() {
        preTime = System.nanoTime()

        // Per the docs:
        // 0x08-0x09 (2) is proximity sensor
        // 0x0A-0x0C (3) is IR sensor
        // 0x0D-0x0F (3) is green
        // 0x10-0x12 (3) is blue
        // 0x13-0x15 (3) is red
        val bytes = device.read(0x08, 14)
        // 0x07 is MAIN_STATUS
        // b3 is LS (light sensor) data status, b0 is ps (prox sensor) status
        // We don't use these here, but they can be useful for implementing a check to see if
        // the sensor values are stale
        // Full docs here: https://docs.broadcom.com/doc/APDS-9151-DS

        posti2cTime = System.nanoTime()

        // Read red, green and blue values
        // Convoluted way of turning 3 bytes into a 20 bit int
        val green = (((bytes[7].toInt() and 0x0F) shl 16) or
                ((bytes[6].toInt() and 0xFF) shl 8) or
                (bytes[5].toInt() and 0xFF))

        // Blue and red are also scaled (idk why but the RevColorSensorV3 class did it so it must be good)
        // To prevent them overflowing, they're normalized
        val blue = Range.clip(
            ((((bytes[10].toInt() and 0x0F) shl 16) or
                    ((bytes[9].toInt() and 0xFF) shl 8) or
                    (bytes[8].toInt() and 0xFF)) * 1.55).toInt(),
            0, limit
        )

        val red = Range.clip(
            ((((bytes[13].toInt() and 0x0F) shl 16) or
                    ((bytes[12].toInt() and 0xFF) shl 8) or
                    (bytes[11].toInt() and 0xFF)) * 1.07).toInt(),
            0, limit
        )

        // normalize to [0, 1] for normalizedColors
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

        // Turns the two proximity bytes into an 11 bit int
        val rawOptical = (((bytes[1].toInt() and 0xFF) shl 8) or (bytes[0].toInt() and 0xFF)) and 0x7FF

        // Distance in CM is stored in `distance`
        distance = DistanceUnit.CM.fromUnit(DistanceUnit.INCH, inFromOptical(rawOptical))

        // Color in HSV is stored in `hsv`
        Color.RGBToHSV((colors.red * 255).toInt(), (colors.green * 255).toInt(), (colors.blue * 255).toInt(), hsv.col)

        postTime = System.nanoTime()

        detectedArtifact = if (distance < 3.0) {
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

        ActiveOpMode.telemetry.run {
            addData("Color Sensor Detected Artifact", detectedArtifact?.name ?: "None")
//            addData("Distance", "%05.2fcm", distance)
//            addData("Hue","%07.4f", hsv.h)
//            addData("Saturation","%07.4f", hsv.s)
//            addData("Value","%07.4f", hsv.v)
//            addData("Total time", "%04.2fms",(postTime - preTime) / 1E6)
//            addData("i2c Read time", "%04.2fms",(posti2cTime - preTime) / 1E6)
//            addData("Calculations time", "%04.2fms",(postTime - posti2cTime) / 1E6)
//            addLine("------------------------------")
        }
    }
}
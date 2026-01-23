package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.hardware.ServoEx
import org.firstinspires.ftc.teamcode.util.normalizeRadians
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

@Configurable
class Turret(): Subsystem {
    companion object {
        const val GEAR_RATIO: Double = 33.0/89.0

        // the offset between the turret's center of rotation
        // and the robot's center of rotation in inches.
        // 75 is measured in mm, then converted to inches by dividing.
        const val TURRET_OFFSET: Double = 80 / 25.4

        // 667.4157303371 total degrees of freedom for turret
        // manually limit it from
        // -180 to 180
        const val TURRET_RANGE: Double = 360.0 // degrees

        const val SERVO_LIMITS = TURRET_RANGE / GEAR_RATIO
        const val SERVO_MIN = 0.5 - ((SERVO_LIMITS / 2) / 1800)
        const val SERVO_MAX = 0.5 + ((SERVO_LIMITS / 2) / 1800)
    }

    private lateinit var servoL: ServoEx
    private lateinit var servoR: ServoEx

    var debugTelemetry = true
    var automatic = true

    private val goalPositions = mapOf(
        20 to Pose(0.0, 143.0),
        24 to Pose(144.0, 143.0)
    )

    var robotPose = Pose()
        set(pose) {
            field = pose
            turretPose = pose
        }

    // DO NOT SET OUTSIDE OF THE SETTER OF ROBOTPOSE
    // EVER
    // DO NOT DO IT
    // BAD
    var turretPose = Pose()
        set(pose) {
            val r = TURRET_OFFSET
            val theta = pose.heading

            val x = r * cos(theta)
            val y = r * sin(theta)

            field = Pose(pose.x - x, pose.y - y, pose.heading)
        }

    // this lazily initialies the goal pose, so if it's not manually set in the opmode, it assumes RED.
    // this has been verified to work.
    val goalPose by lazy { goalPositions[selectedAlliance?.aprilTagID ?: 24] ?: goalPositions.getValue(24) }
    var selectedAlliance: Alliance? = null
    private var bearing = 0.0

    private var position: Double = 0.5
        set(pos) {
            field = pos
            servoL.position = pos
            servoR.position = pos
        }
    // inputs range from -200 to 200
    var angle: Double
        get() = (position * TURRET_RANGE) - TURRET_RANGE / 2
        set(angle) {
//            val clippedAngle = angle.coerceIn(0.0 - (TURRET_RANGE / 2), TURRET_RANGE / 2)
            // scale angle between -180 and 180
            val clippedAngle = if (angle < -180) angle + 360 else if (angle > 180) angle - 360 else angle
            // make it positive, then scale it to 0..1
            val scaledAngle = (clippedAngle + TURRET_RANGE / 2) / TURRET_RANGE

            position = scaledAngle
        }

    override fun initialize() {
        servoL = ServoEx("turretL")
        servoR = ServoEx("turretR")

        servoL.scaleRange(SERVO_MIN, SERVO_MAX)
        servoR.scaleRange(SERVO_MIN, SERVO_MAX)
    }

    override fun periodic() {
        // we assume robot is 0, 0 in a graph.
        // bearing is equal to the angle between the robot and the goal.
        // it is normalized to (-180, 180] degrees, with 0 being right
        bearing = Math.toDegrees(atan2(goalPose.y - turretPose.y, goalPose.x - turretPose.x))

        // normalize robot pose between (-180, 180]
        val robotHeading = Math.toDegrees(if (turretPose.heading > Math.PI) { turretPose.heading - (2 * Math.PI) } else { turretPose.heading } )

        // allows you to do manual control just by setting automatic to false and updating turret.angle
        if (automatic) {
            angle = bearing - robotHeading
        }

        if (debugTelemetry) {
            ActiveOpMode.telemetry.run{
                addData("Turret calculated bearing", bearing)
                addData("Turret robot heading", robotHeading)
                addData("Turret target angle", angle)
                addData("Turret current position", position)
                addData("Distance", goalPose.distanceFrom(turretPose))
                addLine("---------------------------")
            }
        }
    }
}
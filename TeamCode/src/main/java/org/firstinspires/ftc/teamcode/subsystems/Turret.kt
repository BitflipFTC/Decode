package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.ftc.PoseConverter
import com.pedropathing.geometry.PedroCoordinates
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.util.Range
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.PIDController
import org.firstinspires.ftc.teamcode.util.hardware.CRServoEx
import org.firstinspires.ftc.teamcode.util.hardware.ServoEx
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata
import kotlin.math.atan2

@Configurable
class Turret(): Subsystem {
    companion object {
        const val GEAR_RATIO: Double = 33.0/89.0

        // 667.4157303371 total degrees of freedom for turret
        // manually limit it from
        // -240 to 240
        const val TURRET_RANGE: Double = 480.0 // degrees

        const val SERVO_LIMITS = TURRET_RANGE / GEAR_RATIO
        const val SERVO_MIN = 0.5 - ((SERVO_LIMITS / 2) / 1800)
        const val SERVO_MAX = 0.5 + ((SERVO_LIMITS / 2) / 1800)
    }

    private lateinit var servoL: ServoEx
    private lateinit var servoR: ServoEx

    private val goalPositions = mapOf(
        20 to Pose(2.0, 144.0),
        24 to Pose(142.0, 144.0)
    )

    var robotPose = Pose()
    var goalPose = Pose()
        private set
    var selectedAlliance: Alliance? = null
        set(alliance) {
            field = alliance
            // if alliance was not set, just auto set it to red
            goalPose = goalPositions[alliance?.aprilTagID ?: 24] ?: goalPositions.getValue(24)
        }
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
            val clippedAngle = angle.coerceIn(0.0 - (TURRET_RANGE / 2), TURRET_RANGE / 2)
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
        bearing = Math.toDegrees(atan2(goalPose.y - robotPose.y, goalPose.x - robotPose.x))

        // normalize robot pose between (-180, 180]
        val robotHeading = Math.toDegrees(if (robotPose.heading > Math.PI) { robotPose.heading - (2 * Math.PI) } else { robotPose.heading } )

        angle = bearing - robotHeading

        ActiveOpMode.telemetry.addData("Turret calculated bearing", bearing)
        ActiveOpMode.telemetry.addData("Turret robot heading", robotHeading)
        ActiveOpMode.telemetry.addData("Turret target angle", angle)
        ActiveOpMode.telemetry.addData("Turret target position", position)
        ActiveOpMode.telemetry.addLine("---------------------------")
    }
}
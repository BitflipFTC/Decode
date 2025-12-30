package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.hardware.MotorEx;

import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/**
 * Manages the transfer mechanism, which moves artifacts from the spindexer to the shooter.
 * This class uses a PID controller to precisely control the rotation of the transfer motor.
 * The main function is [transferArtifact], which initiates a transfer sequence.
 * The [update] method must be called in a loop to drive the motor to its target position.
 */
@Configurable
public class Transfer implements Subsystem {
    // 1620rpm motor, so
    private final double TICKS_PER_REVOLUTION = 145.1;
    private double targetPosition = 0;

    // amount of times the motor should turn every time it transfers an
    // artifact to the flywheel
    public static int MOTOR_TURNS = 1;

    public static double kP = 0.007;
    public static double kI = 0.03;
    public static double kD = 0.0002;
    public static double maxPower = 0.5;
    public static boolean tuning = true;

    boolean debugTelemetry = true;

    private MotorEx motor = null;

    private final PIDController controller = new PIDController(kP, kI, kD);

    @Override
    public void initialize() {
        controller.setSetPointTolerance(10);
        motor = new MotorEx("transfer").zeroed().brake();
    }

    public void setPower (double power) {
        motor.setPower(power);
    }

    private void setMotorTarget(double ticks) {
        targetPosition = ticks;
    }

    public double getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public void setTargetPosition(double t) {
        targetPosition = t;
    }

    /**
     * Initiates the transfer of an artifact. The motor will turn at least a specified number of times,
     * turning (at most) one additional time in order to return to the nearest rotation
     * and always return to the same spot.
     */
    public void transferArtifact() {
        // set the initial target position (the current position + MOTOR_TURNS rotations)
        double targetPosition = getCurrentPosition() + MOTOR_TURNS * TICKS_PER_REVOLUTION; // ticks

        // convert to revolutions
        double targetRevolutions = targetPosition / TICKS_PER_REVOLUTION; // revolutions

        // determine the difference between where the current target position is and where it should be
        // positive if it is too short of a full revolution
        // ceil ensures that the transfer always does AT LEAST MOTOR_TURNS rotations
        double targetPositionErrorFromRotation = Math.ceil(targetRevolutions) - targetRevolutions;

        // then set a new target that snaps to the nearest rotation
        // adds the difference between the current target and the nearest revolution
        // to ensure the rotation always ends at the same spot
        double targetPositionToNearestRotation = targetPosition + targetPositionErrorFromRotation * TICKS_PER_REVOLUTION;
        setMotorTarget(targetPositionToNearestRotation);
    }

    public void undoTransfer() {
        setMotorTarget(getCurrentPosition() - TICKS_PER_REVOLUTION * MOTOR_TURNS);
    }

    public double getTransferMotorAngle() {
        double degreesPosition = (motor.getCurrentPosition() / TICKS_PER_REVOLUTION) * 360;
        degreesPosition %= 360;
        degreesPosition = (degreesPosition + 360) % 360;
        return degreesPosition;
    }

    /**
     * Updates the transfer mechanism. This method should be called in a loop.
     */
    @Override
    public void periodic() {
        double pidOutput = controller.calculate(getCurrentPosition(), targetPosition);
        motor.setPower(Range.clip(pidOutput, -maxPower, maxPower));

        if (tuning) {
            controller.setCoeffs(kP, kI, kD, 0.0, 0.0);
        }

        if (debugTelemetry) {
            ActiveOpMode.telemetry().addData("Transfer current ticks", getCurrentPosition());
            ActiveOpMode.telemetry().addData("Transfer target ticks", targetPosition);
            ActiveOpMode.telemetry().addData("Transfer at set point", atSetPoint());
            ActiveOpMode.telemetry().addLine("---------------------------");
        }
    }

    public boolean atSetPoint() {
        return controller.atSetPoint();
    }

    public LambdaCommand shootArtifact() {
        return new LambdaCommand()
                .setStart(this::transferArtifact)
                .setIsDone(this::atSetPoint)
                .setName("Shoot artifact")
                .setInterruptible(true)
                .setRequirements(this);
    }
}

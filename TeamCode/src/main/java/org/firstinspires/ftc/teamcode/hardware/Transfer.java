package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PIDController;

/**
 * Manages the transfer mechanism, which moves artifacts from the spindexer to the shooter.
 *
 * This class uses a PID controller to precisely control the rotation of the transfer motor.
 * The main function is [transferArtifact], which initiates a transfer sequence.
 * The [update] method must be called in a loop to drive the motor to its target position.
 */
@Config
public class Transfer {
    private final DcMotorEx motor;
    private boolean transferring;
    private final String configName = "transfer";

    // 1620rpm motor, so
    private final double TICKS_PER_REVOLUTION = 103.8;
    private double targetPosition = 0;

    // amount of times the motor should turn every time it transfers an
    // artifact to the flywheel
    public static int MOTOR_TURNS = 5;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double maxPower = 0.75;
    private final PIDController controller = new PIDController(kP, kI, kD, 0, 0, 1, -1);

    private final Telemetry telemetry;

    public void setPower (double power) {
        motor.setPower(power);
    }

    public Transfer(OpMode opMode) {
        motor = opMode.hardwareMap.get(DcMotorEx.class, configName);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry = opMode.telemetry;
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
        transferring = true;
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
    public void periodic() {
        double pidOutput = controller.calculate(getCurrentPosition(), targetPosition);
        motor.setPower(Range.clip(pidOutput, -maxPower, maxPower));
        controller.setSetPointTolerance(10);
        controller.setCoeffs(kP, kI, kD, 0.0, 0.0);
    }

    public boolean atSetPoint() {
        return controller.atSetPoint();
    }
}

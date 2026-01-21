package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.skeletonarmy.marrow.OpModeManager;

import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.hardware.MotorEx;

/**
 * Manages the transfer mechanism, which moves artifacts from the spindexer to the shooter.
 * This class uses a PID controller to precisely control the rotation of the transfer motor.
 * The main function is [transferArtifact], which initiates a transfer sequence.
 * The [update] method must be called in a loop to drive the motor to its target position.
 */
@Configurable
public class Transfer extends SubsystemBase {
    // 1150rpm motor, so
    private final double TICKS_PER_REVOLUTION = 145.1;
    private double targetPosition = 0;

    // amount of times the motor should turn every time it transfers an
    // artifact to the flywheel
    public static int MOTOR_TURNS = 1;

    public static double kP = 0.005;
    public static double kI = 0.07;
    public static double kD = 0.0002;
    public static double maxPower = 0.67;
    public static boolean tuning = true;

    boolean debugTelemetry = true;

    private final MotorEx motor;

    private final PIDController controller = new PIDController(kP, kI, kD);

    public Transfer() {
        controller.setSetPointTolerance(5.0);
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
        double localTargetPosition = getTargetPosition() + MOTOR_TURNS * TICKS_PER_REVOLUTION; // ticks

        setMotorTarget(localTargetPosition);
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
            OpModeManager.getTelemetry().addData("Transfer current ticks", getCurrentPosition());
            OpModeManager.getTelemetry().addData("Transfer target ticks", targetPosition);
            OpModeManager.getTelemetry().addData("Transfer at set point", atSetPoint());
            OpModeManager.getTelemetry().addLine("---------------------------");
        }
    }

    public boolean atSetPoint() {
        return controller.atSetPoint();
    }

//    public LambdaCommand shootArtifact() {
//        return new LambdaCommand()
//                .setStart(this::transferArtifact)
//                .setIsDone(this::atSetPoint)
//                .setName("Shoot artifact")
//                .setInterruptible(true)
//                .setRequirements(this);
//    }
}

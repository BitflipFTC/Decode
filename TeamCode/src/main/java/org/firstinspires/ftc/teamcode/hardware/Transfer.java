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

    private void returnToStart() {
        double motorAngle = getTransferMotorAngle();
        double error = Math.abs(motorAngle);
        double targetTicksRel = (error / 360) * TICKS_PER_REVOLUTION;
        double targetTicks = getCurrentPosition() + targetTicksRel;
        setMotorTarget(targetTicks);
    }

    public double getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    /**
     * Initiates the transfer of an artifact. The motor will turn a specified number of times
     * and then return to its starting position.
     */
    public void transferArtifact() {
        setMotorTarget(getCurrentPosition() + MOTOR_TURNS * TICKS_PER_REVOLUTION);
        transferring = true;
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
        if (controller.atSetPoint() && transferring) {
            returnToStart();
            transferring = false;
        }
    }

    public boolean atSetPoint() {
        return controller.atSetPoint();
    }
}

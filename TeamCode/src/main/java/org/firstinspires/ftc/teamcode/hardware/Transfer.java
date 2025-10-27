package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.PIDController;
@Config
public class Transfer {
    private final DcMotorEx motor;
    private boolean transferring;
    private final String configName = "transfer";
    // 1150rpm motor, so
    private final double TICKS_PER_REVOLUTION = 103.8;
    private double targetPosition = 0;

    // amount of times the motor should turn every time it transfers an
    // artifact to the flywheel
    public static int MOTOR_TURNS = 5;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    private final PIDController controller = new PIDController(kP, kI, kD, 0, 0, 1, -1);

    public Transfer(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, configName);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    public void transferArtifact() {
        setMotorTarget(getCurrentPosition() + MOTOR_TURNS * TICKS_PER_REVOLUTION);
        transferring = true;
    }

    public double getTransferMotorAngle() {
        double degreesPosition = (motor.getCurrentPosition() / TICKS_PER_REVOLUTION) * 360;
        degreesPosition %= 360;
        degreesPosition = (degreesPosition + 360) % 360;
        return degreesPosition;
    }
    public void update() {
        double pidOutput = controller.calculate(getCurrentPosition(), targetPosition);
        motor.setPower(pidOutput);
        controller.setSetPointTolerance(10);
        controller.setCoeffs(kP, kI, kD, 0.0, 0.0);
        if (controller.atSetPoint() && transferring) {
            returnToStart();
            transferring = false;
        }
    }
}


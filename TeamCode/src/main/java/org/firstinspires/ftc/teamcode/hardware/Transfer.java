package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Transfer {
    private final DcMotorEx motor;
    private final String configName = "transfer";
    // 1150rpm motor, so
    private final double TICKS_PER_REVOLUTION = 145.1;

    // amount of times the motor should turn every time it transfers an
    // artifact to the flywheel
    public static int MOTOR_TURNS = 15;

    public Transfer(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, configName);
        // TODO do rest of motor setup
        // TODO make sure to call stop and reset encoder so the zero value is consistent
    }

    private void setMotorTarget(double ticks) {
        // todo set motor target to this value
    }

    private void returnToStart() {
        // todo: fill this with code that determines the current angle
        // todo: using getTransferAngle(), figures out where it needs to go to be at its 0 point
        // todo: relative to the current interval (modulo by TICKS_PER_REVOLUTION?),
        // todo: and sets the new target position using setMotorTarget()
    }

    public void transferArtifact() {
        // TODO fill this with code that turns the transfer MOTOR_TURNS amount of times,
        // TODO then makes the shortest turn to be fully down
        // TODO make sure to call returnToStart()
    }

    public double getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public double getTransferAngle() {
        // TODO do math here to figure out the current angle of the transfer in
        // TODO the interval [0,360), then return it
        return 0.0;
    }
}

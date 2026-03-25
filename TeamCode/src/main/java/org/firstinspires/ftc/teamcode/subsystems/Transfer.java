package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.util.hardware.MotorEx;

import dev.nextftc.core.subsystems.Subsystem;

/**
 * Manages the transfer mechanism, which is a simple motor. It has two states, on and off.
 */
@Configurable
public class Transfer implements Subsystem {
    private MotorEx motor = null;
    public boolean transferOn = false;
    public boolean transferReverse = false;

    @Override
    public void initialize() {
        motor = new MotorEx("transfer").reverse().zeroed().floa();
    }

    private void setPower (double power) {
        motor.setPower(power);
    }

    /**
     * Initiates the transfer of an artifact. The motor will turn at least a specified number of times,
     * turning (at most) one additional time in order to return to the nearest rotation
     * and always return to the same spot.
     */
    public void on() {
        transferOn = true;
    }

    public void off() {
        transferOn = false;
        transferReverse = false;
    }

    public void reverse() {
        transferReverse = true;
    }

    public void deReverse() {
        transferReverse = false;
    }

    /**
     * Updates the transfer mechanism. This method should be called in a loop.
     */
    @Override
    public void periodic() {
        if (transferOn) {
            motor.setPower((transferReverse ? -1.0 : 1.0));
        } else {
            motor.setPower(0.0);
        }
    }
}
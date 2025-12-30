package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    double Kp = 0.0;
    double Ki = 0.0;
    double Kd = 0.0;

    double reference = 0.0;
    double lastReference = reference;
    double integralSum = 0.0;
    double lastError = 0.0;
    double maxIntegralSum = 0.0;
    double A = 0.8;
    double previousFilterEstimate = 0.0;
    double currentFilterEstimate = 0.0;
    ElapsedTime timer = new ElapsedTime();

    while(setPointIsNotReached) {

        encoderPosition = Motor.getPosotion();
        error = reference - encoderPosition;
        errorChange = (error - lastError);

        currentFilterEstimate = (A * previousFilterEstimate) + (1 - A) * errorChange;
        previousFilterEstimate = currentFilterEstimate;

        derivative = currentFilterEstimate / timer.seconds();

        integralSum = integralSum + (error * timer.seconds());

        if (integralSum > maxIntegralSum) {
            integralSum = maxIntegralSum;
        }

        if (integralSum < -maxIntegralSum) {
            integralSum = -maxIntegralSum;
        }

        if (reference != lastReference) {
            integralSum = 0;
        }

        out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
    }



}

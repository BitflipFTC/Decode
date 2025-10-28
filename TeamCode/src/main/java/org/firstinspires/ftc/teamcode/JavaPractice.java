package org.firstinspires.ftc.teamcode;// package org.firstinspires.ftc.teamcode;
// Explains where file is located

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// Bring in code from FTC SDK

@Disabled
@TeleOp()  // Shows up on Driver Station as OpMode -- @ is an annotation (@ Teleop or @ Autonomous, or @Disabled (compile but won't be shown in list of OpModes))
public class JavaPractice extends OpMode {  // public required for OpModes, class defines class, extends makes whatever comes after a child of OpMode
    @Override // tells compiler we are replacing functionality in parent class
    public void init() {   // void returns nothing, init is name of method
        telemetry.addData("Hello", "World");  // sends data to driver station

        int teamNumber = 16072;
        double motorSpeed = 0.5;
        boolean touchSensorPressed = true;

        telemetry.addData("Team Number", teamNumber);
        telemetry.addData("Motor Speed", motorSpeed);
        telemetry.addData("Touch Sensor", touchSensorPressed);

        int grade = 99;

        telemetry.addData("Grade", grade);


    }


    @Override
    public void loop() {
        int x = 5;
        // x is visible here
        {
            int y = 4;
            // x and y are visible here
        }
        // only x is visible here
    }
}



/* int teamNumber;  (numbers with no decimal---long is larger)
    double motorSpeed;   (practically unlimited floating point numbers)
    boolean touchSensorPressed;  (true or false)

    teamNumber = 16072;
    motorSpeed = 0.5;
    touchSensorPressed = true;

    double MotorSpeed = 0.5;

@Teleop()
    public class UseString extends OpMode {
        @Override
        public void init() {
            String myName = "Alan Smith";

            telemetry.addData("Hello", myName);
        }
 */
package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.ArrayList;

public class ProgrammingBoard {
    private DigitalChannel touchSensor; // type of class: DigitalChannel (FTC SDK), name: touchSensor
    private DcMotor motor;  // variable of type DcMotor with name "motor"
    private double ticksPerRotation;
    private Servo servo;
    private AnalogInput pot; //potentiometer
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    private BNO055IMU imu; // Gyro



    public void init(HardwareMap hwMap) {  // name: hwMap, HardwareMap (FTC SDK), programs get info from config file on robot
        touchSensor = hwMap.get(DigitalChannel.class, "touch sensor"); // assigns to touchSensor the hardware in config file of type DigitalChannel.class with name of "touch sensor."
        touchSensor.setMode(DigitalChannel.Mode.INPUT); // set as INPUT to read from touch sensor
        motor = hwMap.get(DcMotor.class, "motor"); // assigns to motor hardware that is in config file of type DcMotor.class with name of "motor"
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ticksPerRotation = motor.getMotorType().getTicksPerRev();
        servo = hwMap.get(Servo.class, "servo");
        pot = hwMap.get(AnalogInput.class, "pot");
        colorSensor = hwMap.get(ColorSensor.class, "sensor color distance");
        distanceSensor = hwMap.get(DistanceSensor.class, "sensor color distance");
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        // change to default set of parameters go here
        imu.initialize(params);
        servo.setPwmDisable();
    }

    public boolean isTouchSensorPressed() {

        return !touchSensor.getState();
    }
    public void setMotorSpeed(double speed) {  // code outside class can set motor speed
        motor.setPower(speed);  // -1.0 is full backwards, 0.0 is stopped. 1.0 is full forwards
    }
    public double getMotorRotations() {
        return motor.getCurrentPosition() / ticksPerRotation;
    }
    public void setServoPosition(double position) {
        servo.setPosition(position);
    }
    public double getPotAngle() {
        return Range.scale(pot.getVoltage(), 0, pot.getMaxVoltage(), 0, 270); //returns angle to pot--AnalogInput class gives voltage, but it's simpler to use Range.scale to convert between voltage and angle
    }
    public int getAmountRed(){
        return colorSensor.red();
    }
    public double getDistance(DistanceUnit du){

        return distanceSensor.getDistance(du);
    }
    public double getHeading(AngleUnit angleUnit) {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
                                                       AxesOrder.ZYX,
                                                               angleUnit);
        return angles.firstAngle;
    }

    public ArrayList<TestItem> getTests() {
        ArrayList<TestItem> tests = new ArrayList<>();
        tests.add(new TestMotor("PB Motor", 0.5, motor));
        tests.add(new TestAnalogInput("PB Pot", pot, 0, 270));
        return tests;
    }

}

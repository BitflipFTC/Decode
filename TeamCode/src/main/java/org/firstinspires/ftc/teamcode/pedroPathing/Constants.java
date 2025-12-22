package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.34)
            .forwardZeroPowerAcceleration(-55)//todo avg 5 values on full batt
            .lateralZeroPowerAcceleration(-85)//todo avg 5 values on full batt
            .translationalPIDFCoefficients(new PIDFCoefficients(0.06, 0.0, 0.003, 0.04))
            .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0, 0.01, 0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0075, 0, 0.0002, 0.01, 0.6))
            .centripetalScaling(0.0005);


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
            .xVelocity(60.9047)//todo avg 5 values on full batt
            .yVelocity(51.0390461)//todo avg 5 values on full batt
            .rightFrontMotorName("frontright")
            .rightRearMotorName("backright")
            .leftRearMotorName("backleft")
            .leftFrontMotorName("frontleft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(7.46751969)
            .strafePodX(-0.314961)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.8, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
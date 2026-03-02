package org.firstinspires.ftc.teamcode.Pedro;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    private static double deceleration;
    private static double velocity;
    public static double res = 2000/(Math.PI * 31);
    public static double podY = 48.366*0.0394,podX = 48.366*0.0394;
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.8)
            .forwardZeroPowerAcceleration(-30.116402787518567)
            .lateralZeroPowerAcceleration(-55.77951403633623)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.08,0,0.002,0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(0.6,0,0.002,0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.3,0,0.03 ,0.7,0.02))
            .centripetalScaling(0.0008)
            ;
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorEx.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorEx.Direction.REVERSE)
            .xVelocity(42.036494104880994)
            .yVelocity(49.444752910944416);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(podY)
            .strafePodX(podX)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            //.encoderResolution(res,DistanceUnit.MM)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static PathConstraints pathConstraints = new PathConstraints(0.95, 100, 1.5, 0.90);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

}

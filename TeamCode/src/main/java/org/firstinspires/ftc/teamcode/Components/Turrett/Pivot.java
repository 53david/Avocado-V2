package org.firstinspires.ftc.teamcode.Components.Turrett;

import static org.firstinspires.ftc.teamcode.Stuff.TurretPID.Kd;
import static org.firstinspires.ftc.teamcode.Stuff.TurretPID.Ki;
import static org.firstinspires.ftc.teamcode.Stuff.TurretPID.Kp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Components.Odo;
import org.firstinspires.ftc.teamcode.OpModes.Teleop;
import org.firstinspires.ftc.teamcode.Stuff.PIDController;

@Configurable
public class Pivot {

    private static double y=0.33,x = 0.25;
    public static double goalPositionX = 825, goalPositionY = -300;

    public static double ManualOffset =0;

    public static double maxContinuosLimit = Math.PI * 2;
    public static double angleOffset = 450;
    public static double offset = 1.9018;
    public double globalError;
    DcMotorEx rotate;
    GoBildaPinpointDriver pp;
    public double gearRatio = 130.0 / 24.0;
    PIDController pid = new PIDController(Kp,Ki,Kd);
    public Odo odo = new Odo(pp);

    public Pivot (HardwareMap hwMap){
        rotate = hwMap.get(DcMotorEx.class,"rotate");
        rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void update (){

        SparkFunOTOS.Pose2D pos = Odo.getCurrentPosition();
        pos.h = Teleop.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        updateFacingDirection(pos);
        odo.setPosition(new SparkFunOTOS.Pose2D(Odo.getCurrentPosition().x, Odo.getCurrentPosition().y, pos.h));
        odo.update();
    }
    private double fromEncoderToRads() {
        double ticks = rotate.getCurrentPosition() + angleOffset;
        double ticksPerRev = 384.5 * (130.0 / 34.0);
        return ticks * 2.0 * Math.PI / ticksPerRev;
    }
    private double LiniarizedTargetAngle(double targetAngle,double currContinuosAngle){
        double bestdifference = 1e9, bestcontinuosAngle = 1e9;

        double distNormalizedContinuos = Odo.cwDistance(2*Math.PI - offset,targetAngle);

        while(distNormalizedContinuos<=maxContinuosLimit)
        {
            if(bestdifference > Math.abs(distNormalizedContinuos - currContinuosAngle))
            {
                bestdifference = Math.abs(distNormalizedContinuos - currContinuosAngle);
                bestcontinuosAngle = distNormalizedContinuos;
            }
            distNormalizedContinuos+=2*Math.PI;
        }
        if(bestcontinuosAngle == 1e9)
            return 1e9;
        return currContinuosAngle - bestcontinuosAngle;
    }
    public void updateFacingDirection(SparkFunOTOS.Pose2D robotPose) {

        robotPose = new SparkFunOTOS.Pose2D(-robotPose.y, robotPose.x, robotPose.h);

        double robotHeading = robotPose.h;

        double dx = goalPositionX - robotPose.x;
        double dy = goalPositionY - robotPose.y;

        double targetGlobalHeading = Math.atan2(dy, dx);

        double ShouldHaveTurretHeading = targetGlobalHeading - robotHeading + Math.toRadians(ManualOffset);


        double currentTurretRel = fromEncoderToRads();


        double error = LiniarizedTargetAngle(ShouldHaveTurretHeading, currentTurretRel);
        globalError = error;

        if (error == 1e9)
            rotate.setPower(0);
        else
            rotate.setPower(pid.calculatePower(error));
    }
}

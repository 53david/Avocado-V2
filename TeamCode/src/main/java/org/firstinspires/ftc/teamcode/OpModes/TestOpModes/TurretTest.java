package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Components.Odo;
import org.firstinspires.ftc.teamcode.Components.Turret;
import org.firstinspires.ftc.teamcode.Stuff.PIDController;


@TeleOp
@Configurable
@Disabled
public class TurretTest extends LinearOpMode {
    public static double KP = 0;
    public static double KI = 0;
    public static double KD = 0;
    public static double FeedForwardTurret = 0;
    public static double ManualOffset =0;
    public static double goalPositionX = 0;
    public static double goalPositionY = 0;
    public double globalError =0;
    TelemetryManager telemetryM;
    Turret turret;
    Odo odo;
    DcMotorEx rotate,shoot1,shoot2;
    PIDController turretController = new PIDController(KP,KI,KD);
    GoBildaPinpointDriver pp;
    WebcamName webcam;
    @Override
    public void runOpMode(){
        hardwinit();
        waitForStart();
        while(opModeIsActive()){
            turretController = new PIDController(KP,KI,KD);
            telemetryM.update();
            updateFacingDirection(odo.getCurrentPosition());
            if (gamepad1.circleWasPressed()){
                rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }
    private double fromEncoderToRads() {
        double ticks = rotate.getCurrentPosition() + angleOffset;
        double ticksPerRev = 384.5 * (130.0 / 34.0);
        return ticks * 2.0 * Math.PI / ticksPerRev;
    }
    public static double maxContinuosLimit = Math.PI * 2;
    public static double angleOffset = 0;
    public static double offset = 2.269;
    private double CameraOffset(){
        return 0;
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

        robotPose = new SparkFunOTOS.Pose2D(-robotPose.y,robotPose.x, robotPose.h);

        double robotHeading = robotPose.h;

        double dx = goalPositionX - robotPose.x;
        double dy = goalPositionY - robotPose.y;

        double targetGlobalHeading = Math.atan2(dy, dx);

        double ShouldHaveTurretHeading = targetGlobalHeading - robotHeading + Math.toRadians(ManualOffset);


        double currentTurretRel = fromEncoderToRads();

        telemetryM.addData("currentTurretRel",currentTurretRel);

        double error = LiniarizedTargetAngle(ShouldHaveTurretHeading,currentTurretRel);
        globalError = error;

        telemetryM.addData("error",error);
        telemetryM.addData("currticks", rotate.getCurrentPosition());
        telemetryM.update();
        if(error == 1e9)
            rotate.setPower(0);
        else
            rotate.setPower(turretController.calculatePower(error) + Math.signum(error) * FeedForwardTurret);
    }
    public void hardwinit(){
        rotate = hardwareMap.get(DcMotorEx.class,"rotate");
        pp = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        odo = new Odo(pp);
    }
}

package org.firstinspires.ftc.teamcode.Components;


import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm1;
import static org.firstinspires.ftc.teamcode.OpModes.Teleop.prevgm1;
import static org.firstinspires.ftc.teamcode.OpModes.TestOpModes.BetterFormula.rpm;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.vel1;
import static org.firstinspires.ftc.teamcode.Stuff.Sigma.Ka;
import static org.firstinspires.ftc.teamcode.Stuff.Sigma.Kd;
import static org.firstinspires.ftc.teamcode.Stuff.Sigma.Ki;
import static org.firstinspires.ftc.teamcode.Stuff.Sigma.Kp;
import static org.firstinspires.ftc.teamcode.Stuff.Sigma.Ks;
import static org.firstinspires.ftc.teamcode.Stuff.Sigma.Kv;
import android.util.Size;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OpModes.Teleop;
import org.firstinspires.ftc.teamcode.Stuff.ShooterConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

@Configurable
public class Turret {
    public static double KP = 1;
    public static double KD = 0.02;
    double allianceID = 20;
    public static long exposure = 2;
    public static int gain = 200;
    public static double Voltage = 0;
    double target =0;
    WebcamName webcam;
    double fx = 807.567, fy = 807.567, cx = 345.549, cy = 267.084;
    public static double ManualOffset =0;
    public static double FeedForwardTurret = 0;
    public static double x = 20;
    GoBildaPinpointDriver pp;
    public AprilTagProcessor tagProcessor;
    public VisionPortal visionPortal;
    public static DcMotorEx rotate, shoot1, shoot2;
    public TelemetryManager telemetryM;
    public double globalError;

    PIDController pid = new PIDController(0.01,0,0.0002);
    org.firstinspires.ftc.teamcode.Stuff.PIDController turretController = new org.firstinspires.ftc.teamcode.Stuff.PIDController(1.2,0,0.05);
    PIDController controller = new PIDController(Kp,Ki,Kd);

    public enum AllienceState {
        RED,
        BLUE,
    }
    public enum State{
        Manual,
        Auto,
    }
    AllienceState state = AllienceState.BLUE;
    State bstate = State.Manual;
    public static double goalPositionX = 825, goalPositionY = -300;

    public Turret(DcMotorEx rotate, DcMotorEx shoot1, DcMotorEx shoot2, TelemetryManager telemetryM, WebcamName webcam,GoBildaPinpointDriver pp) {
        this.rotate = rotate;
        this.telemetryM = telemetryM;
        this.shoot2 = shoot2;
        this.shoot1 = shoot1;
        this.webcam = webcam;
        this.pp = pp;
        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoot2.setDirection(DcMotorSimple.Direction.REVERSE);

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setLensIntrinsics(fx, fy, cx, cy)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(webcam)
                .setCameraResolution(new Size(800, 600))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .build();

        PanelsCameraStream.INSTANCE.startStream(visionPortal, 10);
        visionPortal.stopStreaming();

    }
    private double fromEncoderToRads() {
        double ticks = rotate.getCurrentPosition() + angleOffset;
        double ticksPerRev = 384.5 * (130.0 / 34.0);
        return ticks * 2.0 * Math.PI / ticksPerRev;
    }
    public static double maxContinuosLimit = Math.PI * 2;
    public static double angleOffset = 450;
    public static double offset = 1.9018;
    private double CameraOffset(){

        for(AprilTagDetection tag:tagProcessor.getDetections()){
            if (tag.id == allianceID) {
               return Math.toRadians(tag.ftcPose.bearing);
            }
        }
        return 1e9;
    }
    private double DistanceOffset(){

        for(AprilTagDetection tag:tagProcessor.getDetections()){
            if (tag.id == allianceID) {
                return tag.ftcPose.range;
            }
        }
        return 1e9;
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

            telemetryM.addData("currentTurretRel", currentTurretRel);

            double error = LiniarizedTargetAngle(ShouldHaveTurretHeading, currentTurretRel);
            globalError = error;

            telemetryM.addData("error", error);
            telemetryM.addData("currticks", rotate.getCurrentPosition());
            if (error == 1e9)
                rotate.setPower(0);
            else
                rotate.setPower(turretController.calculatePower(error) + Math.signum(error) * FeedForwardTurret);
            telemetryM.addData("X",pp.getPosX(DistanceUnit.MM));
    }
    public static double p1RPM = 321, p2RMP = 653;
    public static double dist1 = 214, dist2 = 765;

    public void update() {
        controller = new PIDController(Kp,Ki,Kd);
        Odo odo =new Odo(pp);
        telemetryM.addData("distance",Odo.distance()*0.0394);
        telemetryM.addData("Velocity",ShooterConstants.fwVel(Odo.distance()*0.0394));
        telemetryM.addData("heading",pp.getHeading(AngleUnit.RADIANS));
        telemetryM.addData("vel",shoot2.getVelocity());
        odo.update();

        vel1 = controller.calculate(-shoot2.getVelocity(),rpm);
        vel1 += Kv * rpm + Ks;
        vel1 *= Voltage;
        shoot1.setPower(vel1);
        shoot2.setPower(vel1);
        telemetryM.update();
        AllienceUpdate();
        TurretUpdate();
    }
    public static double getVelo(){
        return Math.abs(shoot2.getVelocity());
    }
    public void AllienceUpdate(){
        switch (state){
            case BLUE:
                goalPositionX = 825;goalPositionY = -250;
                if (gm1.square && gm1.square!=prevgm1.square)
                    state = AllienceState.RED;
                allianceID = 20;
                break;
            case RED:
                goalPositionX = -2600; goalPositionY = -250;
                if (gm1.square && gm1.square!=prevgm1.square)
                    state = AllienceState.BLUE;
                allianceID = 24;
                break;
        }
    }
    public void TurretUpdate(){
        Odo odo =new Odo(pp);
        switch (bstate){
            case Manual:
                target +=gm1.right_stick_x * 40;
                rotate.setPower(pid.calculate(rotate.getCurrentPosition(),target));
                if (gm1.circle && prevgm1.circle!=gm1.circle) {
                    bstate = State.Auto;
                    odo.setPosition(new SparkFunOTOS.Pose2D(0,0, pp.getHeading(AngleUnit.RADIANS)));
                    //rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    //rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                if (gm1.triangle && prevgm1.triangle!=gm1.triangle){
                    rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    target = 0;
                }
                telemetryM.addLine("Manual");
                break;
            case Auto:
                SparkFunOTOS.Pose2D pos = Odo.getCurrentPosition();
                pos.h = Teleop.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                updateFacingDirection(pos);
                odo.setPosition(new SparkFunOTOS.Pose2D(Odo.getCurrentPosition().x, Odo.getCurrentPosition().y, pos.h));
                if (gm1.circle && prevgm1.circle!=gm1.circle) {
                    bstate = State.Manual;
                }
                telemetryM.addLine("Auto");
                break;
        }
    }

}


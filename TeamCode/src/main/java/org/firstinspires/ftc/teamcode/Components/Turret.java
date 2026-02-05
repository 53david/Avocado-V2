package org.firstinspires.ftc.teamcode.Components;


import static org.firstinspires.ftc.teamcode.Stuff.TurretPID.D;
import static org.firstinspires.ftc.teamcode.Stuff.TurretPID.I;
import static org.firstinspires.ftc.teamcode.Stuff.TurretPID.P;

import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Stuff.PIDController;
import org.firstinspires.ftc.teamcode.Stuff.ShooterConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
public class Turret{
    private static final double P = 0.02, I = 0,D = 0.002;
    private static final double Kp = 0, Ki = 0, Kd = 0;
    public double a = 0, target = 0;
    public DcMotorEx rotate,shoot1,shoot2;
    public AprilTagProcessor tagProcessor;
    public VisionPortal visionPortal;
    TelemetryManager telemetryM;
    double fx=807.567, fy=807.567, cx=345.549 , cy=267.084;
    public PIDController pid = new PIDController(P,I,D);
    public PIDController pidFw = new PIDController(Kp,Ki,Kd);
    public PIDCoefficients coef = new PIDCoefficients(P,I,D);
    WebcamName webcam;
    public Turret(DcMotorEx rotate, DcMotorEx shoot1, DcMotorEx shoot2, WebcamName webcam, TelemetryManager telemetryM){
        this.rotate=rotate;
        this.telemetryM=telemetryM;
        this.webcam=webcam;
        this.shoot2 = shoot2;
        this.shoot1 = shoot1;

        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setLensIntrinsics(fx,fy,cx,cy)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(webcam)
                .setCameraResolution(new Size(800, 600))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .build();
    }

    public void update(){
            turretTrack();
            flyWh();
    }
    public void flyWh(){
        for(AprilTagDetection tag:tagProcessor.getDetections()){
            if (tag.id == 20) {
                shoot1.setPower(pid.calculatePower(ShooterConstants.fwVel(tag.ftcPose.range)));
                shoot2.setPower(pid.calculatePower(ShooterConstants.fwVel(tag.ftcPose.range)));
            }
        }
    }
    public void turretTrack(){

        ArrayList<AprilTagDetection> detections = tagProcessor.getDetections();
        boolean IsTagSeen = false;
        for(AprilTagDetection tag:tagProcessor.getDetections()){
            if (tag.id == 20) {
                rotate.setPower(pid.calculatePower(tag.ftcPose.bearing));
                coef = new PIDCoefficients(P, I, D);
                pid.setPidCoefficients(coef);
                IsTagSeen = true;
            }
        }
        if(!IsTagSeen){
            rotate.setPower(0);
        }

        telemetryM.update();
    }



}

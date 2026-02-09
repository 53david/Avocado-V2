package org.firstinspires.ftc.teamcode.Components;


import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm1;
import static org.firstinspires.ftc.teamcode.OpModes.Teleop.prevgm1;

import android.util.Size;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorGoBildaPinpoint;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Stuff.PIDController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

@Configurable
public class Turret{
    GoBildaPinpointDriver gobilda;
    private static double P = 0, I = 0,D = 0;
    public static double Kp = 14.5;
    public static double Kf = 5.4;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double vel = 1200;
    public static long exposure = 2;
    public static int gain = 200;
    int allianceID = 20;
    public double a = 0, target = 0;
    public DcMotorEx rotate,shoot1,shoot2;
    public AprilTagProcessor tagProcessor;
    public VisionPortal visionPortal;
    TelemetryManager telemetryM;
    double fx=807.567, fy=807.567, cx=345.549 , cy=267.084;
    public PIDController pid = new PIDController(P,I,D);
    public PIDFCoefficients coef = new PIDFCoefficients(Kp, Ki, Kd, Kf);
    WebcamName webcam1;
    public enum AllienceState{
        RED,
        BLUE,
    }
    AllienceState state = AllienceState.BLUE;
    public Turret(DcMotorEx rotate, DcMotorEx shoot1, DcMotorEx shoot2, WebcamName webcam, TelemetryManager telemetryM){
        this.rotate=rotate;
        this.telemetryM=telemetryM;
        this.webcam1=webcam;
        this.shoot2 = shoot2;
        this.shoot1 = shoot1;
        this.gobilda = gobilda;
        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoot1.setDirection(DcMotorSimple.Direction.REVERSE);

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

        PanelsCameraStream.INSTANCE.startStream(visionPortal,30);
        visionPortal.stopStreaming();
        shoot1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,coef);
        shoot2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,coef);

    }

    public void update(){
        turretTrack();
        FlyWh();
        reset();
        allienceUpdate();
        gobilda.update();
        telemetryM.addData("pos",rotate.getCurrentPosition());
        telemetryM.addData("X",gobilda.getEncoderX());
        telemetryM.addData("Y",gobilda.getEncoderY());
    }
    public void FlyWh(){
        shoot1.setVelocity(vel);
        telemetryM.addData("Velocity",shoot2.getVelocity());
    }
    public void turretTrack(){


        boolean IsTagSeen = false;
        for(AprilTagDetection tag:tagProcessor.getDetections()){
            if (tag.id == allianceID) {
                rotate.setPower(pid.calculatePower(tag.ftcPose.bearing));
                IsTagSeen = true;
            }
        }
        if(!IsTagSeen){
            rotate.setPower(0);
        }

        telemetryM.update();
    }
    public void reset(){
        if (gm1.squareWasPressed()) {
            rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotate.setTargetPosition(0);
            rotate.setPower(1);
        }
    }
    public void allienceUpdate(){
        switch (state){
            case RED:
                allianceID = 24;
                telemetryM.addLine("RED");
                break;
            case BLUE:
                allianceID = 20;
                telemetryM.addLine("BLUE");
                break;
        }
        if (gm1.dpadRightWasPressed()) {
            state = AllienceState.BLUE;
        }
        if (gm1.dpadDownWasPressed()) {
            state = AllienceState.RED;
        }

    }

}

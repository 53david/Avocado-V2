package org.firstinspires.ftc.teamcode.Components;


import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm1;
import static org.firstinspires.ftc.teamcode.OpModes.Teleop.prevgm1;

import android.util.Size;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
    public PIDFController pidfController = new PIDFController(Kp,Ki,Kd,Kf);
    WebcamName webcam1;
    Odo odo = new Odo(gobilda,telemetryM);
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
        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoot1.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void update(){
        FlyWh();
        allienceUpdate();
        telemetryM.addData("pos",rotate.getCurrentPosition());
    }
    public void FlyWh(){
        shoot1.setPower(pidfController.calculate(shoot1.getVelocity(),1200));
        pidfController = new PIDFController(Kp,Ki,Kd,Kf);
        telemetryM.addData("Velocity",shoot1.getVelocity());
    }
    public void allienceUpdate(){

        switch (state){
            case RED:
                telemetryM.addData("target Angle",odo.thetaRed());
                telemetryM.update();
                rotate.setPower(pid.calculatePower(odo.thetaRed()));
                if (gm1.dpadRightWasPressed()){
                    state = AllienceState.BLUE;
                }
                break;
            case BLUE:
                telemetryM.addData("target Angle",odo.thetaBlue());
                telemetryM.update();
                rotate.setPower(pid.calculatePower(odo.thetaBlue()));
                if (gm1.dpadRightWasPressed()){
                    state = AllienceState.RED;
                }
                break;
        }

    }

}

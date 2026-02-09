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
import org.firstinspires.ftc.teamcode.Stuff.ShooterConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

@Configurable
public class Turret{
    private static double P = 0, I = 0,D = 0;
    public static double Kp = 0;
    public static double Kf = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public DcMotorEx rotate,shoot1,shoot2;
    public TelemetryManager telemetryM;
    public GoBildaPinpointDriver gobilda;
    public PIDController pid = new PIDController(P,I,D);
    public PIDFController pidfController = new PIDFController(Kp,Ki,Kd,Kf);
    Odo odo = new Odo(gobilda);
    public enum AllienceState{
        RED,
        BLUE,
    }
    AllienceState state = AllienceState.BLUE;
    public Turret(DcMotorEx rotate, DcMotorEx shoot1, DcMotorEx shoot2, TelemetryManager telemetryM){
        this.rotate=rotate;
        this.telemetryM=telemetryM;
        this.shoot2 = shoot2;
        this.shoot1 = shoot1;
        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoot1.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void update(){
        switch (state){
            case RED:
                telemetryM.addLine("RED");
                telemetryM.addData("target Angle",odo.thetaRed());
                telemetryM.update();
                rotate.setPower(pid.calculatePower(odo.thetaRed()));
                shoot2.setPower(pidfController.calculate(shoot2.getVelocity(), ShooterConstants.fwVel(odo.deltaRED())));

                if (gm1.dpadRightWasPressed()){
                    state = AllienceState.BLUE;
                }
                break;
            case BLUE:
                telemetryM.addLine("BLUE");
                telemetryM.addData("target Angle",odo.thetaBlue());
                telemetryM.update();
                rotate.setPower(pid.calculatePower(odo.thetaBlue()));
                shoot2.setPower(pidfController.calculate(shoot2.getVelocity(),ShooterConstants.fwVel(odo.deltaBLUE())));

                if (gm1.dpadRightWasPressed()){
                    state = AllienceState.RED;
                }
                break;
        }
    }
    }



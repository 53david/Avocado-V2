package org.firstinspires.ftc.teamcode.Wrappers;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class Initializer {

    public static DcMotorEx intakeMotor;
    public static DcMotorEx leftFront;
    public static DcMotorEx rightFront;
    public static DcMotorEx leftBack;
    public static DcMotorEx rightBack;
    public static DcMotorEx rotate;
    public static DcMotorEx shoot1;
    public static DcMotorEx shoot2;
    public static WebcamName webcam;
    public static Servo transfer;
    public static void start(HardwareMap hwMap){
        intakeMotor = hwMap.get(DcMotorEx.class,"intake");
        transfer = hwMap.get(Servo.class,"transfer");
        leftFront = hwMap.get(DcMotorEx.class,"leftFront");
        rightFront = hwMap.get(DcMotorEx.class,"rightFront");
        rightBack = hwMap.get(DcMotorEx.class,"rightBack");
        leftBack = hwMap.get(DcMotorEx.class,"leftBack");
        shoot1 = hwMap.get(DcMotorEx.class,"shoot1");
        shoot2 = hwMap.get(DcMotorEx.class,"shoot2");
        rotate = hwMap.get(DcMotorEx.class,"rotate");
        webcam = hwMap.get(WebcamName.class,"Webcam 1");
    }
}

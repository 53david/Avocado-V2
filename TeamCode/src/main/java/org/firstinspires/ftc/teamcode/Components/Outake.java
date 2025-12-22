package org.firstinspires.ftc.teamcode.Components;


import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm1;
import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm2;
import static org.firstinspires.ftc.teamcode.OpModes.Teleop.prevgm1;
import static org.firstinspires.ftc.teamcode.OpModes.Teleop.prevgm2;
import static org.firstinspires.ftc.teamcode.OpModes.Teleop.vision;

import static org.firstinspires.ftc.teamcode.OpModes.Teleop.prevnext;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpModes.Teleop;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class Outake {
    private Telemetry telemetry; ElapsedTime timer = new ElapsedTime();
    private DcMotorEx shoot1, shoot2, rotate;
    private Servo servo;
    enum State{
        CLOSE,
        FAR,
    }

    State state = State.CLOSE;

    public Outake(DcMotorEx shoot1, DcMotorEx shoot2, DcMotorEx rotate, Servo servo, Telemetry telemetry) {
        this.shoot1 = shoot1;
        this.shoot2 = shoot2;
        this.rotate = rotate;
        this.servo = servo;
        this.telemetry = telemetry;
        shoot1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shoot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void update() {
        if (gm2.circle) {
            servo.setPosition(0);
        } else {
            servo.setPosition(0.95);
        }
    }

    public void shooter() {
        switch (state){
            case CLOSE:
                telemetry.addLine("CLOSE");
                if (gm2.right_trigger==prevgm2.right_trigger && gm2.right_trigger>0)
                    state=State.FAR;
                shoot1.setPower(-0.65);
                shoot2.setPower(0.65);
                break;
            case FAR:
                telemetry.addLine("FAR");
                if (gm2.triangle==prevgm2.square && gm2.right_trigger==0)
                    state=State.CLOSE;
                shoot1.setPower(-0.75);
                shoot2.setPower(0.75);
                break;
        }
        telemetry.update();
        rotate.setPower(gm2.right_stick_x*0.33);
    }
    public void shoot() {
        if (timer.milliseconds()>1500 && timer.milliseconds()<2000){
            servo.setPosition(0);
        }
        else if (timer.milliseconds()>2000 && timer.milliseconds()<2500){
            servo.setPosition(0.95);
        }
        else if (timer.milliseconds()>3000){
            timer.reset();
        }
    }
    public void run(){
        shoot1.setPower(-0.6);
        shoot2.setPower(0.6);
    }

}

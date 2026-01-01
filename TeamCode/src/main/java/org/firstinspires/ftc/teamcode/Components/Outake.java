package org.firstinspires.ftc.teamcode.Components;


import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outake {
    private Telemetry telemetry; ElapsedTime timer = new ElapsedTime();
    double P = 0, F = 0, power; double velocity;
    public PIDFCoefficients coefsf = new PIDFCoefficients(P,0,0,F);
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
        shoot1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoot1.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void init(HardwareMap hwMap){

        shoot1 = hwMap.get(DcMotorEx.class,"shoot1");
        shoot2 = hwMap.get(DcMotorEx.class,"shoot2");
        rotate = hwMap.get(DcMotorEx.class,"rotate");
        servo = hwMap.get(Servo.class,"transfer");

    }
    public void update() {
        if (gm2.circleWasPressed()) {
            servo.setPosition(0);
        }
        if(servo.getPosition()==0){
            servo.setPosition(0.95);
        }
    }

    public void shooter() {
        shoot1.setPower(power);
        shoot2.setPower(power);
        switch (state){
            case CLOSE:
                telemetry.addLine("CLOSE");
                if (gm2.squareWasPressed())
                    state=State.FAR;
                power = 0.60;
                break;
            case FAR:
                telemetry.addLine("FAR");
                if (gm2.squareWasPressed())
                    state=State.CLOSE;
                power = 0.75;
                break;
        }
        telemetry.update();
        rotate.setPower(gm2.right_stick_x*0.33);

    }
    public void activate(){
        if (timer.milliseconds()<500)
            servo.setPosition(0);
        else {
            servo.setPosition(0.95);
            timer.reset();
        }
    }



}

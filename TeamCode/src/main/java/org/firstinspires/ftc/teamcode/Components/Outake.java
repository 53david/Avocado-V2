package org.firstinspires.ftc.teamcode.Components;


import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm1;
import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm2;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outake {
    private Telemetry telemetry;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();
    double power;
    double velocity;
    boolean ok = true; int nr = 0;
    private DcMotorEx shoot1, shoot2, rotate; CRServo p;
    private final static double P = 0 ,F = 0, I=0;
    private PIDFCoefficients coef1 = new PIDFCoefficients(P,I,0,F);
    private Servo servo;

    enum State {
        POW0,
        POW1,
        POW2,
        POW3,
        POW4,
    };
    enum ShootState{
        IDLE,
        SHOOT,
    }
    State state = State.POW2;
    ShootState shootState = ShootState.IDLE;
    public Outake(DcMotorEx shoot1, DcMotorEx shoot2, DcMotorEx rotate,Telemetry telemetry) {
        this.shoot1 = shoot1;
        this.shoot2 = shoot2;
        this.rotate = rotate;

        this.telemetry = telemetry;
        shoot1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoot1.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void shooter() {
        telemetry.update();
        switch (state){
            case POW0:
                telemetry.addLine("POW0");
                shoot1.setPower(0);
                shoot2.setPower(0);
                if (gm1.cross){
                    state = State.POW1;
                }
                break;
            case POW1:
                telemetry.addLine("POW1");
                shoot1.setPower(0.45);
                shoot2.setPower(0.45);
                if (gm1.cross){
                    state = State.POW2;
                }
                break;
            case POW2:
                telemetry.addLine("POW2");
                shoot1.setPower(0.65);
                shoot2.setPower(0.65);
                if (gm1.cross){
                    state =State.POW3;
                }
                break;
            case POW3:
                telemetry.addLine("POW3");
                shoot1.setPower(0.75);
                shoot2.setPower(0.75);
                if (gm1.cross){
                    state =State.POW4;
                }
                break;
            case POW4:
                telemetry.addLine("POW4");
                shoot1.setPower(1);
                shoot2.setPower(1);
                if (gm1.cross){
                    state =State.POW0;
                }
                break;

        }

    }



}



package org.firstinspires.ftc.teamcode.Components;


import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm1;
import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm2;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.D;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.F;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.I;
import static org.firstinspires.ftc.teamcode.Stuff.FlyWheelPIDF.P;

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
    PIDFCoefficients pidfcoef = new PIDFCoefficients(P,I,D,F);
    private Servo servo;
    enum State {
        IDLE,
        CLOSE,
        FAR,
    };
    State state;
    public Outake(DcMotorEx shoot1, DcMotorEx shoot2) {
        this.shoot1 = shoot1;
        this.shoot2 = shoot2;

        shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfcoef);
        shoot2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfcoef);
        shoot1.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public void shooter() {
        telemetry.update();
        switch (state){
            case CLOSE:
                shoot1.setVelocity(900);
                shoot2.setVelocity(900);
            case FAR:
                shoot1.setVelocity(1500);
                shoot2.setVelocity(1500);
        }

    }

}



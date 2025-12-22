package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm1;
import static org.firstinspires.ftc.teamcode.OpModes.Teleop.prevgm1;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Stuff.PIDController;

public class Storage {
    private CRServo revolver;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime time = new ElapsedTime();
    double p = 1.0, t=2000; boolean ok; Gamepad gamepad2;
    boolean next = false, prevnext = false;
    DcMotorEx encoder;
    public PIDController turner;
    public double Target = 0;

    int n=0;
    public enum IntakeState{
        IDLE,
        ACTIVE,

    };
    public enum StorageState{
        BALL1,
        BALL2,
        BALL3,
    }
    StorageState state1 = StorageState.BALL1; private DcMotorEx motor;
    IntakeState state2 = IntakeState.IDLE;

    public Storage (CRServo revolver,DcMotorEx encoder, DcMotorEx motor){
        this.revolver = revolver;
        this.encoder = encoder;
        this.motor = motor;
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turner = new PIDController(0,0,0);
        revolver.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double FromTicksToDegrees(){
        return encoder.getCurrentPosition() / 8192.0 * Math.PI * 2.0;
    }

    public void Turn120(){
        Target -= Math.PI * 2 / 3;
        turner.setTargetPosition(Target);
    }

    public void update()
    {
        revolver.setPower(turner.calculatePower(FromTicksToDegrees()));
    }
    public void Index(){
        if (timer.milliseconds()>3000) {
            Turn120();
            timer.reset();
        }

    }
    public void TakeBall(){
        switch (state2){
            case IDLE:
                Index();
                if (prevgm1.right_bumper==gm1.right_bumper && gm1.right_bumper){
                    state2=IntakeState.ACTIVE; time.reset();

                }
                break;
            case ACTIVE:
                if (time.milliseconds()>500) {
                    Turn120(); timer.reset();
                    state2 = IntakeState.IDLE;
                }
                break;
        }

    }
    public boolean IsStorageSpinning(){
        return Math.abs(Target-FromTicksToDegrees()) < Math.toRadians(5) && encoder.getVelocity() < 20;
    }

}
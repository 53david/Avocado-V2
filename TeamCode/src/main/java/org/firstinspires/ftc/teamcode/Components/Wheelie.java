package org.firstinspires.ftc.teamcode.Components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm2;

@Configurable
public class Wheelie {
    private static int x = 0;
    CRServo wh1,wh2;
    ElapsedTime timer = new ElapsedTime();
    public enum State{
        IDLE,
        ASCEND,
        DESCEND,
    };
    State state = State.IDLE;
    public Wheelie(CRServo wh1,CRServo wh2){
        this.wh1 = wh1;
        this.wh2 = wh2;
    }
    public void update(){
        switch (state){
            case IDLE:
                Idle();
                break;
            case ASCEND:
                timer.reset();
                Ascend();
                break;
            case DESCEND:
                timer.reset();
                Descend();
                break;
        }
        if (x==0){
            state = State.IDLE;
        }
        else if (x==1){
            state = State.ASCEND;
        }
        else if (x==2){
            state = State.DESCEND;
        }
    }
    public void Ascend(){
        if(timer.milliseconds()<5000){
            wh1.setPower(1);
            wh2.setPower(1);
        }
    }
    public void Descend(){
        if(timer.milliseconds()<5000){
            wh1.setPower(-1);
            wh2.setPower(-1);
        }
    }
    public void Idle(){
        wh1.setPower(0);
        wh2.setPower(0);
    }
}

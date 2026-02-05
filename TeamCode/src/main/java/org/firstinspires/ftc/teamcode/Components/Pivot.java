package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm1;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Pivot {

    private static double y=0.33,x = 0.25;
    DcMotorEx Pivot;
    int pos = 0; Servo transfer;
    public Pivot (DcMotorEx Pivot, Servo transfer){
        this.Pivot = Pivot;
        this.transfer = transfer;
        Pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Pivot.setDirection(DcMotorSimple.Direction.REVERSE);
        Pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void update (){
        if (gm1.cross){
            transfer.setPosition(x);
        } else transfer.setPosition(y);

    }
}

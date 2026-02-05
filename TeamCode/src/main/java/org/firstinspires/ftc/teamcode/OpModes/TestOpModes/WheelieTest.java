package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Components.Wheelie;

@TeleOp(name = "Wheelie Test")
public class WheelieTest extends LinearOpMode {
    CRServo wh1,wh2;
    Wheelie wheelie;

    @Override
    public void runOpMode(){
        hardwinit();
        waitForStart();
        while (opModeIsActive()){
            wheelie.update();
        }
    }
    public void hardwinit(){
        wh1 = hardwareMap.get(CRServo.class,"wh1");
        wh2 = hardwareMap.get(CRServo.class,"wh2");
        wheelie = new Wheelie(wh1,wh2);
    }
}

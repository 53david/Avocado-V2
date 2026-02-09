package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class LeaveBlue extends LinearOpMode {
    ElapsedTime time = new ElapsedTime();
    DcMotorEx m1,m2,m3,m4; //apple silicon 67
    @Override
        public void runOpMode(){
            hardwinit();
            waitForStart();
            while (opModeIsActive()){
                if (time.milliseconds()<1500) {
                    m1.setPower(-1);
                    m2.setPower(1);
                    m3.setPower(1);
                    m4.setPower(-1);
                }
                else {
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                }
            }
        }
        public void hardwinit(){
        time.reset();
            m1 = hardwareMap.get(DcMotorEx.class,"rightFront");
            m2 = hardwareMap.get(DcMotorEx.class,"leftFront");
            m3 = hardwareMap.get(DcMotorEx.class,"rightBack");
            m4 = hardwareMap.get(DcMotorEx.class,"leftBack");
            m2.setDirection(DcMotorSimple.Direction.REVERSE);
            m4.setDirection(DcMotorSimple.Direction.REVERSE);
        }
}

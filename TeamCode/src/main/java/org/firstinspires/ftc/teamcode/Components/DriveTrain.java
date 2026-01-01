package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.OpModes.Teleop.gm1;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;


public class DriveTrain {
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    boolean ok;
    public DriveTrain(DcMotorEx leftFront, DcMotorEx rightFront, DcMotorEx leftBack, DcMotorEx rightBack) {

        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack=leftBack;
        this.rightBack = rightBack;
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }
    public void init(HardwareMap hwMap){
        leftFront = hwMap.get(DcMotorEx.class,"leftFront");
        rightFront = hwMap.get(DcMotorEx.class,"rightFront");
        leftBack = hwMap.get(DcMotorEx.class,"leftBack");
        rightBack = hwMap.get(DcMotorEx.class,"rightBack");
        MotorConfigurationType m= leftFront.getMotorType();
        m.setAchieveableMaxRPMFraction(1);
        leftFront.setMotorType(m);
        rightFront.setMotorType(m);
        leftBack.setMotorType(m);
        rightBack.setMotorType(m);

    }

    public void update() {

        double y = -gm1.left_stick_y;
        double x = gm1.left_stick_x*1.1;
        double rx = gm1.right_trigger-gm1.left_trigger;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx)/ denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x- rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }
    public void StrafeRight (){
        leftFront.setPower(-1);
        leftBack.setPower(1);
        rightFront.setPower(1);
        rightBack.setPower(-1);
    }
    public void StrafeLeft(){
        leftFront.setPower(1);
        leftBack.setPower(-1);
        rightFront.setPower(-1);
        rightBack.setPower(1);
    }
    public void BackWards(){
        leftFront.setPower(-1);
        leftBack.setPower(-1);
        rightFront.setPower(-1);
        rightBack.setPower(-1);
    }


}